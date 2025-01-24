function ekf = ekf_conf(param)

import casadi.*

% use casadi for calculting jacobians
ekf = {};
ekf.state_size = 69;   
% The State of the system is as follows:
% State X
%   - position          (1:3)   body position
%   - velocity          (4:6)   body velocity
%   - acceleration      (7:9)   body acceleration
%   - euler angle       (10:12) body orientation Tait-Bryan
%   - angular velocity  (13:15) body angular velocity
%   - foot1 pos         (16:18) FL foot position
%   - foot1 vel         (19:21) FL foot velocity
%   - foot2 pos         (22:24) FR foot position
%   - foot2 vel         (25:27) FR foot velocity
%   - foot3 pos         (28:30) RL foot position
%   - foot3 vel         (31:33) RL foot velocity
%   - foot4 pos         (34:36) RR foot position
%   - foot4 vel         (37:39) RR foot velocity
%   - body acc bias     (40:42)
%   - body gyro bias    (43:45)
%   - foot1 acc bias    (46:48)
%   - foot1 gyro bias   (49:51)
%   - foot2 acc bias    (52:54)
%   - foot2 gyro bias   (55:57)
%   - foot3 acc bias    (58:60)
%   - foot3 gyro bias   (61:63)
%   - foot4 acc bias    (64:66)
%   - foot4 gyro bias   (67:69)

% Control U
%   - a1      (1:3)     foot 1 IMU acceleration (already in body frame)
%   - a2      (4:6)   foot 2 IMU acceleration (already in body frame)
%   - a3      (7:9)   foot 3 IMU acceleration (already in body frame)
%   - a4      (10:12)   foot 4 IMU acceleration (already in body frame)
%   - t       (13)      time 

ekf.meas_size = 71; 
ekf.control_size = 13;


% get process jacobians
s_xk = casadi.MX.sym('X_k', ekf.state_size);    % define a symbolic variable representing the current state
s_uk = casadi.MX.sym('Uk', ekf.control_size);   % define a symbolic variable representing the current control input
s_dt = casadi.MX.sym('dt', 1);   % symbolic variable representing the time step
s_f = dyn_rk4(s_xk , s_uk, s_dt, @process);   % EKF process: symbolic expression of the state update
s_F = jacobian(s_f, s_xk);   % Jacobian of the new state with respect to old state 
s_B = jacobian(s_f, s_uk);   % Jacobian of the new state with respect to control 
ekf.f = Function('process',{s_xk, s_uk, s_dt},{s_f});
ekf.df = Function('process_jac',{s_xk, s_uk, s_dt},{s_F});
ekf.db = Function('control_jac',{s_xk, s_uk, s_dt},{s_B});

% get measurement jacobians
s_yaw = casadi.MX.sym('yaw', 1);
s_phi = casadi.MX.sym('phi', 12);  % define a symbolic variable representing the joint angles
s_dphi = casadi.MX.sym('dphi', 12);   % define a symbolic variable representing the joint velocities
s_ddphi = casadi.MX.sym('ddphi', 12);   % define a symbolic variable representing the joint accelerations
s_w_f = casadi.MX.sym('w_f', 12);   % define a symbolic variable representing the feet IMU angular velocities
s_a_f = casadi.MX.sym('a_f', 12);   % define a symbolic variable representing the feet IMU linear accelerations
s_w_dot_f = casadi.MX.sym('dw_f', 12);   % define a symbolic variable representing the feet IMU angular velocities
s_w_b = casadi.MX.sym('w_b', 3);   % define a symbolic variable representing the body IMU angular velocities
s_a_b = casadi.MX.sym('w_a', 3);   % define a symbolic variable representing the body IMU linear accelerations
s_r = measurement(s_xk, s_phi, s_dphi, s_ddphi, s_w_f, s_a_f, s_w_dot_f, s_w_b, s_a_b, s_yaw, param);   % EKF measurement: symbolic expression of the expected measurement residual
s_R = jacobian(s_r, s_xk);   % Jacobian of the measurement function with respect to state
ekf.r = Function('meas',{s_xk, s_phi, s_dphi, s_ddphi, s_w_f, s_a_f, s_w_dot_f, s_w_b, s_a_b, s_yaw},{s_r});
ekf.dr = Function('meas_jac',{s_xk, s_phi, s_dphi, s_ddphi, s_w_f, s_a_f, s_w_dot_f, s_w_b, s_a_b, s_yaw},{s_R});

end