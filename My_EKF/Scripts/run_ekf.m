function [state, param] = run_ekf(data, param)

total_steps = size(data.acc_b_IMU.Time,1);
total_start_idx = param.data_start_idx;
total_end_idx =  total_steps-200;
param.data_end_idx = total_end_idx;
N = total_end_idx - total_start_idx + 1; 

ekf = ekf_conf(param);

% init foot pos and vel
foot_pos_vel_list = zeros(6*param.num_leg,1);
if param.has_mocap == 1
    init_pos = data.pos_mocap.Data(total_start_idx,:)';
else
    init_pos = [0;0;param.init_body_height];
end
init_euler = zeros(3,1);
R_bw = euler_to_rot(init_euler);
phi = data.j_ang.Data(total_start_idx,:)';
for i = 1:param.num_leg
    angle = phi((i-1)*3+1:(i-1)*3+3);
    p_fk = p_fk_func(angle,param.lc,param.leg(:,i));
    foot_pos_vel_list((i-1)*6+1:(i-1)*6+3) = R_bw*p_fk +init_pos;
end

x0 = [
    init_pos;
    zeros(3,1);
    zeros(3,1);
    init_euler;
    zeros(3,1);
    foot_pos_vel_list;
    zeros(3,1);
    zeros(3,1);
    zeros(3,1);
    zeros(3,1);
    zeros(3,1);
    zeros(3,1);
    zeros(3,1);
    zeros(3,1);
    zeros(3,1);
    zeros(3,1)];

x_list = zeros(ekf.state_size, N+1);
x_list(:,1) = x0;


P0 = param.init_cov*eye(ekf.state_size);
P0(40:69,40:69) = param.init_bias_cov*eye(30);
cov_list = zeros(ekf.state_size,ekf.state_size, N+1);
cov_list(:,:,1) = P0;

for idx=total_start_idx:total_end_idx
    k = idx-total_start_idx+1;
    dt = data.om_b_IMU.Time(idx+1) - data.om_b_IMU.Time(idx);
    % convert foot acc from foot frame to body frame using FK rotation
    j_angs = data.j_ang.Data(idx,:)';
    accel_IMUs = [
                    data.acc_fl_IMU.Data(idx,:)';
                    data.acc_fr_IMU.Data(idx,:)';
                    data.acc_rl_IMU.Data(idx,:)';
                    data.acc_rr_IMU.Data(idx,:)';
                 ];
    gyro_IMUs = [
                    data.om_fl_IMU.Data(idx,:)';
                    data.om_fr_IMU.Data(idx,:)';
                    data.om_rl_IMU.Data(idx,:)';
                    data.om_rr_IMU.Data(idx,:)';
                 ];
    accel_IMU_bs = zeros(3*param.num_leg,1);   % foot IMU acceleration in body frame
    gyro_IMU_bs = zeros(3*param.num_leg,1);    % foot IMU angular velocity in body frame
    for leg_id=1:param.num_leg
        leg_j_angs = j_angs((leg_id-1)*3+1:(leg_id-1)*3+3);
        R_fb = R_fb_func(leg_j_angs);
        accel_IMU_bs((leg_id-1)*3+1:(leg_id-1)*3+3) = R_fb*param.R_fs{leg_id}*accel_IMUs((leg_id-1)*3+1:(leg_id-1)*3+3);
        gyro_IMU_bs((leg_id-1)*3+1:(leg_id-1)*3+3) = R_fb*param.R_fs{leg_id}*gyro_IMUs((leg_id-1)*3+1:(leg_id-1)*3+3);
    end
    uk = [accel_IMU_bs;
            dt];

    x01 = full(ekf.f(x_list(:,k), uk, dt));
    F = full(ekf.df(x_list(:,k), uk, dt));
    B = full(ekf.db(x_list(:,k), uk, dt));
    
    % Process Noise Covariance Q1
    ekf.Q1 = diag([param.proc_n_pos* dt *ones(3,1); % pos x y z
                         param.proc_n_vel * dt *ones(3,1); % vel x y z
                         param.proc_n_acc * dt *ones(3,1); % acc x y z
                         param.proc_n_ang *  dt *ones(3,1); % angle x y z
                         param.proc_n_om *  dt *ones(3,1); % ang vel x y z
                         repmat(...
                            [param.proc_n_foot_pos * dt*ones(3,1);  % foot1 pos x y z
                             param.proc_n_foot_vel  * dt*ones(3,1)],4,1);   % foot1 vel  x y z        
                         param.proc_n_ba *  dt *ones(3,1);    % acc bias random walk
                         param.proc_n_bg *  dt *ones(3,1);    % gyro bias random walk
                         param.proc_n_foot1_ba *  dt *ones(3,1);    % foot1 acc bias random walk
                         param.proc_n_foot1_bg *  dt *ones(3,1);    % foot1 gyro bias random walk
                         param.proc_n_foot2_ba *  dt *ones(3,1);    % foot2 acc bias random walk
                         param.proc_n_foot2_bg *  dt *ones(3,1);    % foot2 gyro bias random walk
                         param.proc_n_foot3_ba *  dt *ones(3,1);    % foot3 acc bias random walk
                         param.proc_n_foot3_bg *  dt *ones(3,1);    % foot3 gyro bias random walk
                         param.proc_n_foot4_ba *  dt *ones(3,1);    % foot4 acc bias random walk
                         param.proc_n_foot4_bg *  dt *ones(3,1)]);  % foot4 gyro bias random walk
                         
    % Control Input Noise Covariance Q2 
    ekf.Q2 = diag([param.ctrl_n_foot1_acc * dt *ones(3,1);   % foot 1 IMU acceleration
                         param.ctrl_n_foot2_acc * dt *ones(3,1);   % foot 2 IMU acceleration
                         param.ctrl_n_foot3_acc * dt *ones(3,1);   % foot 3 IMU acceleration
                         param.ctrl_n_foot4_acc * dt *ones(3,1);   % foot 4 IMU acceleration
                         0]);  
           
    ck = double(data.foot_contact.Data(idx,:)); 
    num_meas = 16; % number of residual equation for each leg

    % Measurement Noise Covariance R
    ekf.R = diag([ repmat([param.meas_n_fk_pos * ones(3,1); % forward kinematic
                           param.meas_n_lo_vel * ones(3,1); % leg odometry
                           param.meas_n_om_formula * ones(3,1);  % foot omega equation
                           param.meas_n_acc_formula * ones(3,1);  % foot acceleration equation
                           param.meas_n_zero_vel * ones(3,1) % contact mode estimation 
                           param.meas_n_foot_height],4,1);         
                         param.meas_n_om_body * ones(3,1);    % body IMU angular velocity
                         param.meas_n_acc_body * ones(3,1);     % body IMU linear acceleration
                         param.meas_n_yaw]);    
       
    for i = 1:param.num_leg
        if (param.mipo_use_md_test_flag == 0)
            ekf.R((i-1)*num_meas+13:(i-1)*num_meas+15,(i-1)*num_meas+13:(i-1)*num_meas+15) = ...
                (1 + (1 - ck(i)) * 1e5)*param.meas_n_zero_vel *dt*eye(3);
            ekf.R((i-1)*num_meas+16,(i-1)*num_meas+16) = ...
                (1 + (1 - ck(i)) * 1e5)*param.meas_n_foot_height *dt;
        else
            ekf.R((i-1)*num_meas+13:(i-1)*num_meas+15,(i-1)*num_meas+13:(i-1)*num_meas+15) = ...
                param.meas_n_zero_vel*eye(3);
            ekf.R((i-1)*num_meas+16,(i-1)*num_meas+16) = ...
                param.meas_n_foot_height;
        end
    end
    
    % Covariance Prediction
    P01 = F*cov_list(:,:,k)*F' + ekf.Q1 + B*ekf.Q2*B';

    hat_om_dot_b_b = zeros(1,12);
    hat_om_b_IMU = data.om_b_IMU.Data(idx,:)';
    hat_acc_b_IMU = data.acc_b_IMU.Data(idx,:)';
    hat_phi = data.j_ang.Data(idx,:)';
    hat_dphi = data.j_vel.Data(idx,:)';
    hat_ddphi = data.joint_acc.Data(idx,:)';
    for i = 1:param.num_leg
        hat_om_dot_b_b((i-1)*3+1:(i-1)*3+3) = (data.om_dot_b_b(:,idx,i));
    end
    %
    hat_yaw = data.orient_mocap_euler.Data(idx,3)';
    %
    y = full(ekf.r(x01, hat_phi, hat_dphi, hat_ddphi, gyro_IMU_bs, accel_IMU_bs, hat_om_dot_b_b', hat_om_b_IMU, hat_acc_b_IMU, hat_yaw)); % Residual
    H = full(ekf.dr(x01, hat_phi, hat_dphi, hat_ddphi, gyro_IMU_bs, accel_IMU_bs, hat_om_dot_b_b', hat_om_b_IMU, hat_acc_b_IMU, hat_yaw)); % Jacobian of Measurement
    
    % Residual Covariance 
    S = H*P01*H' + ekf.R;

    
    mask = ones(ekf.meas_size,1);
    % if (param.mipo_use_md_test_flag == 1)
    %     for i = 1:param.num_leg
    %         seg_mes = y((i-1)*num_meas+13:(i-1)*num_meas+15);
    %         seg_S = S((i-1)*num_meas+13:(i-1)*num_meas+15,(i-1)*num_meas+13:(i-1)*num_meas+15);
    %         MD = sqrt(seg_mes'*inv(seg_S)*seg_mes);
    %         if MD > 1
    %             mask((i-1)*num_meas+13:(i-1)*num_meas+15) = zeros(3,1);
    %         end
    %     end
    % end
    mask = logical(mask);

    % Kalman Gain
    K  = P01 * H(mask,:)' * inv(S(mask,mask));

    % State Update
    x_list(:,k+1) = x01 - K*y(mask); 

    % Covariance Update
    cov_list(:,:,k+1) = ( eye(ekf.state_size)-K*H(mask,:))*P01;
    cov_list(:,:,k+1) = (cov_list(:,:,k+1) + cov_list(:,:,k+1)')/2;
end

state = x_list;
end