function ekf = ekf_conf(param)

import casadi.*
% use casadi for calculting jacobians

ekf = {};
ekf.state_size = 69;   
ekf.meas_size = 70; 
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
s_phi = casadi.MX.sym('phi', 12);  % define a symbolic variable representing the joint angles
s_dphi = casadi.MX.sym('dphi', 12);   % define a symbolic variable representing the joint velocities
s_ddphi = casadi.MX.sym('ddphi', 12);   % define a symbolic variable representing the joint accelerations
s_w_f = casadi.MX.sym('w_f', 12);   % define a symbolic variable representing the feet IMU angular velocities
s_a_f = casadi.MX.sym('a_f', 12);   % define a symbolic variable representing the feet IMU linear accelerations
s_w_dot_f = casadi.MX.sym('dw_f', 12);   % define a symbolic variable representing the feet IMU angular velocities
s_w_b = casadi.MX.sym('w_b', 3);   % define a symbolic variable representing the body IMU angular velocities
s_a_b = casadi.MX.sym('w_a', 3);   % define a symbolic variable representing the body IMU linear accelerations
s_r = measurement(s_xk, s_phi, s_dphi, s_ddphi, s_w_f, s_a_f, s_w_dot_f, s_w_b, s_a_b, param);   % EKF measurement: symbolic expression of the expected measurement residual
s_R = jacobian(s_r, s_xk);   % Jacobian of the measurement function with respect to state
ekf.r = Function('meas',{s_xk, s_phi, s_dphi, s_ddphi, s_w_f, s_a_f, s_w_dot_f, s_w_b, s_a_b},{s_r});
ekf.dr = Function('meas_jac',{s_xk, s_phi, s_dphi, s_ddphi, s_w_f, s_a_f, s_w_dot_f, s_w_b, s_a_b},{s_R});

end