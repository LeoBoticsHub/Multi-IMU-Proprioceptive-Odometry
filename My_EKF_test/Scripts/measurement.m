function meas_residual = measurement(x, phi, dphi, ddphi, om_IMU_f, acc_IMU_f, om_dot_b_b, om_IMU_b, acc_IMU_b, param)

    if ~iscolumn(x)
        x = x';
    end

pos = x(1:3);
vel = x(4:6);
acc_body = x(7:9);
euler = x(10:12);
ang_vel = x(13:15);
R_bw = euler_to_rot(euler); % rotation matrix from body to world frame 
foot_pos = [x(16:18); x(25:27); x(34:36); x(43:45)];
foot_vel = [x(19:21); x(28:30); x(37:39); x(46:48)];
foot_acc = [x(22:24); x(31:33); x(40:42); x(49:51)];
ba = x(52:54);      
bg = x(55:57);      

n_meas_leg = 15; % For now there is no slip detection (it will add 3 residual for each leg)
meas_residual = zeros(66,1,class(x)); % setting to zero the residuals

for i = 1:param.num_leg % iteration for all legs
    j_ang = phi((i-1)*3+1:(i-1)*3+3); % joint angle
    j_vel = dphi((i-1)*3+1:(i-1)*3+3); % joint velocity
    j_acc = ddphi((i-1)*3+1:(i-1)*3+3); % joint acceleration
    foot_ba = x((i-1)*6+46:(i-1)*6+48); % foot acc bias
    foot_bg = x((i-1)*6+49:(i-1)*6+51); % foot gyro bias
    p_fk = p_fk_func(j_ang,param.lc,param.leg(:,i)); % forward kinematic: relative position of the foot in the body frame with respect to the body
    J_vel = J_vel_func(j_ang,param.lc,param.leg(:,i)); % velocity jacobian: derivative of the forward kinematic with respect to the three joint angles t1,t2,t3
    J_vel_dot = J_vel_dot_func(j_ang,j_vel,param.lc,param.leg(:,i)); % forward kinematic Hessian: derivative of the velocity jacobian with respect to the three joint angles t1,t2,t3
    leg_v = -(J_vel*j_vel+skew(ang_vel)*p_fk); % body velocity in world frame from leg odometry
    J_w = J_omega_func(j_ang); % angular velocity jacobian
    w_f = om_IMU_f((i-1)*3+1:(i-1)*3+3); % foot IMU angular velocity in body frame
    a_f = acc_IMU_f((i-1)*3+1:(i-1)*3+3); % foot IMU linear acceleration in body frame
    w_dot_b = om_dot_b_b((i-1)*3+1:(i-1)*3+3);

    meas_residual((i-1)*n_meas_leg+1:(i-1)*n_meas_leg+3) = ...
        p_fk - R_bw'*(foot_pos((i-1)*3+1:(i-1)*3+3) - pos); 

    meas_residual((i-1)*n_meas_leg+4:(i-1)*n_meas_leg+6) = ...
        leg_v - R_bw'*(vel - foot_vel((i-1)*3+1:(i-1)*3+3));  
    
    w_b = w_f-foot_bg - J_w*j_vel; % Body angular velocity with my equation
    
    meas_residual((i-1)*n_meas_leg+7:(i-1)*n_meas_leg+9) = ...
        w_b - ang_vel;

    meas_residual((i-1)*n_meas_leg+10:(i-1)*n_meas_leg+12) = ...
        a_f-foot_ba - (J_vel*j_acc+J_vel_dot*j_vel + 2*skew(w_b)*J_vel*j_vel+skew(w_b)*skew(w_b)*p_fk+skew(w_dot_b)*p_fk) - R_bw'*(acc_body+[0;0;9.8]); 
  
    meas_residual((i-1)*n_meas_leg+13:(i-1)*n_meas_leg+15) = ...
        a_f - foot_ba - R_bw'*(foot_acc((i-1)*3+1:(i-1)*3+3)+[0;0;9.8]);
   
end

meas_residual(61:63) = ...
        om_IMU_b-bg - ang_vel;

meas_residual(64:66) = ...
        acc_IMU_b-ba - R_bw'*(acc_body+[0;0;9.8]);

end