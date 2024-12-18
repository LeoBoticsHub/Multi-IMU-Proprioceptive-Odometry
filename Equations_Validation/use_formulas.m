% function to calculate body angular velocity and linear acceleration from
% robot kinematic and foot IMU sensors.

% Setting the indexes
const;
steps = size(resampled_data.acc_b_IMU.Time,1);
init_idx = 1; % choose the best starting index based on dataset
end_idx =  steps;
N = end_idx - init_idx + 1; 

%% Omega Body in body frame
% Preallocate Variables
om_b_b=zeros(3,N,4); % Body Angular Velocity in Body Frame 
a_b_b=zeros(3,N,4); % Body Linear Acceleration in Body Frame

% Iteration for each instant
for idx=init_idx:end_idx % current step index
    k = idx-init_idx+1; % step index scaled to start at 1
    phik = resampled_data.j_ang.Data(idx,:)'; % joint angles data for all legs
    dphik = resampled_data.j_vel.Data(idx,:)'; % joint angular velocities for all legs 
    omega_f_f_IMUs = [
                    resampled_data.om_fl_IMU.Data(idx,:)';
                    resampled_data.om_fr_IMU.Data(idx,:)';
                    resampled_data.om_rl_IMU.Data(idx,:)';
                    resampled_data.om_rr_IMU.Data(idx,:)';
                 ]; % foot absolute angular velocity in foot frame
    omega_f_b_IMUs = zeros(3,1);    % foot absolute angular velocity in body frame
    omega_fk_f_b = zeros(3,1);    % foot relative angular velocity in body frame

    % iteration for each leg
    for leg_id=1:4
        idx_range = (leg_id-1)*3+1:(leg_id-1)*3+3; 
        l_phik = phik(idx_range);     % joint angles data for a leg
        l_dphik = dphik(idx_range);     % joint angular velocity data for a leg
        R_bf = R_bf_func(l_phik);     % rotation matrix from foot to body
        omega_f_b_IMUs = R_bf*param.R_fs{leg_id}*omega_f_f_IMUs(idx_range); 
        J_omega = J_omega_func(l_phik);    % angular velocity Jacobian
        omega_fk_f_b = J_omega*l_dphik;
        om_b_b(:, k, leg_id)= omega_f_b_IMUs-omega_fk_f_b;   % body angular velocity in body frame  
    end
end
%% Compute the acceleration
om_dot_b_b=zeros(3,N,4); % Body Angular Acceleration in Body Frame
% compute the body angular acceleration
[b,g] = sgolay(3,11); 
for leg_id = 1:4
    for p = 1:3
      om_dot_b_b(p, :, leg_id) = conv(om_b_b(p, :, leg_id), factorial(1)/(-dt)^1 * g(:,2), 'same');
    end
end

for idx=init_idx:end_idx 
    k = idx-init_idx+1; % current step index
    phik = resampled_data.j_ang.Data(idx,:)'; % joint angles data for all legs
    dphik = resampled_data.j_vel.Data(idx,:)'; % joint angular velocities for all legs 
    ddphik = resampled_data.joint_acc.Data(idx,:)'; % joint angular acceleration for all legs 
    acc_f_f_IMUs = [
                    resampled_data.acc_fl_IMU.Data(idx,:)';
                    resampled_data.acc_fr_IMU.Data(idx,:)';
                    resampled_data.acc_rl_IMU.Data(idx,:)';
                    resampled_data.acc_rr_IMU.Data(idx,:)';
                 ]; % foot absolute linear acceleration in foot frame
    acc_f_b_IMU = zeros(3,1);    % foot absolute linear acceleration in body frame
    acc_fk_f_b = zeros(3,1);      % foot relative linear acceleration in body frame
    % iteration for each leg
    for leg_id=1:4
        idx_range = (leg_id-1)*3+1:(leg_id-1)*3+3; 
        l_phik = phik(idx_range);     % joint angles data for a leg
        l_dphik = dphik(idx_range);     % joint angular velocity data for a leg
        l_ddphik = ddphik(idx_range);   % % joint angular acceleration data for a leg
        R_bf = R_bf_func(l_phik);     % rotation matrix from foot to body
        acc_f_b_IMU = R_bf*param.R_fs{leg_id}*(acc_f_f_IMUs(idx_range));
        J_vel = J_vel_func(l_phik,param.lc,param.leg(:,leg_id));      % linear velocity Jacobian
        p_fk = p_fk_func(l_phik,param.lc,param.leg(:,leg_id));   % forward kinematics: foot position relative to body in body frame
        J_vel_dot = J_vel_dot_func(l_phik,l_dphik,param.lc,param.leg(:,leg_id)); % derivative of linear velocity jacobian in a 3x3 matrix
        v_f = J_vel*l_dphik;     % forward kinematic: foot velocity relative to body in body frame
        acc_fk_f_b = J_vel*l_ddphik+J_vel_dot*l_dphik;
        a_b_b(:, k, leg_id) = acc_f_b_IMU-(acc_fk_f_b+2*skew(om_b_b(:, k, leg_id))*v_f+skew(om_b_b(:, k, leg_id))*skew(om_b_b(:, k, leg_id))*p_fk+skew(om_dot_b_b(:, k, leg_id))*p_fk);
    end
    
end


%% Comparizon
% Preallocation of variable for the averaging
om_b_b_avg = zeros(3, N);
a_b_b_avg = zeros(3, N);

% Average calculation 
for k = 1:N
    om_b_b_avg(:, k) = mean(om_b_b(:, k, :), 3);
    a_b_b_avg(:, k) = mean(a_b_b(:, k, :), 3);
end

time_vector = resampled_data.om_b_IMU.Time(init_idx:end_idx);
w=resampled_data.om_b_IMU.Data(:,1:3)';
a=resampled_data.acc_b_IMU.Data(:,1:3)';

% % Might need some compensation for the referance fram
% w=zeros(3,steps);
% a=zeros(3,steps);
% w(1,:)=-resampled_data.om_b_IMU.Data(:,1)';
% a(1,:)=-resampled_data.acc_b_IMU.Data(:,1)';
% w(2,:)=resampled_data.om_b_IMU.Data(:,2)';
% a(2,:)=resampled_data.acc_b_IMU.Data(:,2)';
% w(3,:)=resampled_data.om_b_IMU.Data(:,3)';
% a(3,:)=resampled_data.acc_b_IMU.Data(:,3)';


% % Compute the moving mean of om_b_b_avg along the time dimension (columns)
% om_b_b_avg = movmean(om_b_b_avg, 5, 2);
% 
% % Compute the moving mean of a_b_b_avg along the time dimension (columns)
% a_b_b_avg = movmean(a_b_b_avg,7, 2);

% Angular Velocity Comparizon
axis={'x','y','z'};
figure;
for i = 1:3
    subplot(3, 1, i);
    plot(time_vector, om_b_b_avg(i, :), 'b', 'DisplayName', 'Calculated');
    hold on;
    grid on
    plot(time_vector, w(i, init_idx:end_idx), 'r--', 'DisplayName', 'IMU Body');
    xlabel('Time (s)');
    ylabel(['\omega_', axis{i}, ' (rad/s)']);
    legend;
    title(['Body Angular Velocity - Axis ', axis{i}]);
end

% Linear Acceleration Comparizon
figure;
for i = 1:3
    subplot(3, 1, i);
    plot(time_vector, a_b_b_avg(i, :), 'b', 'DisplayName', 'Calculated');
    hold on;
    grid on
    plot(time_vector, a(i, init_idx:end_idx), 'r--', 'DisplayName', 'IMU Body');
    xlabel('Time (s)');
    ylabel(['a_', axis{i}, ' (m/s^2)']);
    legend;
    title(['Body Linear Acceleration - Axis ', axis{i}]);
end
