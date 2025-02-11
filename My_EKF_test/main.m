% cleaning
clear all
close all
clc

%% Comment Section
% Contact Model: use contact flag.
% Added body linear acceleration and body angular velocity to the state

%% Initialize workspace 
const;
addpath("Rotations");
addpath("Functions");
addpath("Scripts");
addpath("casadi");
%% Read ros2bag file
get_sensor_data;

%% Use equations
[om_b_b, a_b_b, om_dot_b_b, om_b_b_avg, a_b_b_avg] = my_equations(resampled_data, param);
resampled_data.om_b_b = om_b_b;
resampled_data.a_b_b = a_b_b;
resampled_data.om_dot_b_b = om_dot_b_b;
resampled_data.om_b_b_avg = om_b_b_avg;
resampled_data.a_b_b_avg = a_b_b_avg;
%% Initialize parameters
param.has_mocap = 1;
param.data_start_idx = 200;
param.init_body_height = 0.3;
param.init_cov = 1e-5;
param.init_bias_cov = 1e-4;

% Process noise parameters
param.proc_n_pos = 1e-5;
param.proc_n_vel = 1e-8;
param.proc_n_acc = 0.05;
param.proc_n_ang = 1e-7;
param.proc_n_om = 1e-5;
param.proc_n_foot_pos = 0.001;
param.proc_n_foot_vel = 1e-6; 
param.proc_n_foot_acc = 0.05; 
param.proc_n_ba = 1e-7;
param.proc_n_bg = 1e-8;
param.proc_n_foot1_ba = 1e-7;  
param.proc_n_foot2_ba = 1e-7;  
param.proc_n_foot3_ba = 1e-7;  
param.proc_n_foot4_ba = 1e-7;   
param.proc_n_foot1_bg = 1e-8;  
param.proc_n_foot2_bg = 1e-8;  
param.proc_n_foot3_bg = 1e-8;  
param.proc_n_foot4_bg = 1e-8; 

% Measurement noise parameters
param.meas_n_fk_pos = 0.0001;
param.meas_n_lo_vel = 0.001;
param.meas_n_foot_acc = 0.001;
param.meas_n_om_formula = 0.01;
param.meas_n_acc_formula = 0.5;
param.meas_n_om_body = 0.001;
param.meas_n_acc_body = 0.05;

%% Run EKF
tune=0;
tic;
[state, param] = run_ekf(resampled_data, param);
simulation_time = toc
plots2;

%% Tuning
tune=1;
for t = [1e-6,1e-7,1e-8,1e-9,1e-10]%[1, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8]
    param.proc_n_ba = t;
    for i = [1e-6,1e-7,1e-8,1e-9,1e-10]%[1, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8]
        param.proc_n_bg = i;
        tic;
        [state, param] = run_ekf(resampled_data, param);
        simulation_time = toc
        plots2;
    end
end
