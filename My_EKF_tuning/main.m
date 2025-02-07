% cleaning
clear all
close all
clc

%% Comment Section
% The simulation is conducted by examining a subset of the total data.
% The constant parameter are taken from the Gazebo Simulation xacro
% The derivatives are calculated using Savitzky Golay Filter
% The measurement residual related to the foot height is working
% No Yaw angle correction
% Contact Model: use contact flag.

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
param.pivoting_model = 1; % 1 means use pivoting model 
param.md = 0; % 1 means use malanobis distance; 0 means contact flag
param.data_start_idx = 200;
param.init_body_height = 0.3;

param.init_cov = 1e-5;
param.init_bias_cov = 1e-4;

% Process noise parameters
param.proc_n_pos_xy = 1e-5;
param.proc_n_pos_z = 1e-5;
param.proc_n_vel_xy = 1e-8;
param.proc_n_vel_z = 1e-8;
param.proc_n_acc_xy = 0.045;
param.proc_n_acc_z = 0.35;
param.proc_n_ang_xy = 1e-8;
param.proc_n_ang_z = 1e-8;
param.proc_n_om_x = 1e-4;
param.proc_n_om_y = 1e-3;
param.proc_n_om_z = 1e-2;
param.proc_n_foot_pos_xy =1e-3;
param.proc_n_foot_pos_z =1e-3;
param.proc_n_foot_vel_xy =0.002;%1e-5;
param.proc_n_foot_vel_z =0.01;%2.5e-5;
param.proc_n_ba = 5e-6;
param.proc_n_bg = 1.6e-5;
param.proc_n_foot_ba = 5e-7;   
param.proc_n_foot_bg = 5e-8;

% Control noise parameters
param.ctrl_n_foot1_acc = 1e-5;
param.ctrl_n_foot2_acc = 1e-5;
param.ctrl_n_foot3_acc = 1e-5;
param.ctrl_n_foot4_acc = 1e-5;

% Measurement noise parameters
param.meas_n_fk_pos_xy = 1e-5;
param.meas_n_fk_pos_z = 0.001;
param.meas_n_lo_vel_xy = 0.001;
param.meas_n_lo_vel_z = 0.001;
param.meas_n_zero_vel_xy = 0.01;
param.meas_n_zero_vel_z = 0.01;
param.meas_n_om_formula_xz = 0.01;
param.meas_n_om_formula_y = 0.01;
param.meas_n_acc_formula = 0.5;
param.meas_n_om_body_xy = 0.001;
param.meas_n_om_body_z = 0.001;
param.meas_n_acc_body_xy = 0.05;
param.meas_n_acc_body_z = 0.05;
param.meas_n_foot_height = 0.001;


%% Run EKF
tic;
[state, param] = run_ekf(resampled_data, param);
simulation_time = toc
plots2;

%% Run EKF

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