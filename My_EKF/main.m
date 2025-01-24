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

% Rotation matrix from IMU to foot frame)
param.R_fs = {[-1  0  0;
               0  0 -1;
               0 -1  0], 
             [-1   0   0; 
               0   0   1;
               0   1   0],
             [-1  0  0;
               0  0 -1;
               0 -1  0],
             [-1   0  0; 
               0   0  1;
               0   1  0]};

%% Use equations
[om_b_b, a_b_b, om_dot_b_b, om_b_b_avg, a_b_b_avg] = my_equations(resampled_data, param);
resampled_data.om_b_b = om_b_b;
resampled_data.a_b_b = a_b_b;
resampled_data.om_dot_b_b = om_dot_b_b;
resampled_data.om_b_b_avg = om_b_b_avg;
resampled_data.a_b_b_avg = a_b_b_avg;
%% Initialize parameters
param.has_mocap = 1;
param.mipo_use_foot_ang_contact_model = 1; % 0 means use zero velocity model
param.mipo_use_md_test_flag = 0; % 0 means use contact flag
param.data_start_idx = 200;
param.init_body_height = 0.3;

param.init_cov = 1e-4;
param.init_bias_cov = 1e-4;

% Process noise parameters
param.proc_n_pos = 0.005;
param.proc_n_vel = 0.001;
param.proc_n_acc = 0.0005;
param.proc_n_ang = 1e-7;
param.proc_n_om = 1e-5;
param.proc_n_foot_pos = 1e-4;
param.proc_n_foot_vel = 0.5;   
param.proc_n_ba = 1e-4;
param.proc_n_bg = 1e-5;
param.proc_n_foot1_ba = 1e-4;  
param.proc_n_foot2_ba = 1e-4;  
param.proc_n_foot3_ba = 1e-4;  
param.proc_n_foot4_ba = 1e-4;   
param.proc_n_foot1_bg = 1e-5;  
param.proc_n_foot2_bg = 1e-5;  
param.proc_n_foot3_bg = 1e-5;  
param.proc_n_foot4_bg = 1e-5; 

% Control noise parameters
param.ctrl_n_foot1_acc = 1e-3;
param.ctrl_n_foot2_acc = 1e-3;
param.ctrl_n_foot3_acc = 1e-3;
param.ctrl_n_foot4_acc = 1e-3;

% Measurement noise parameters
param.meas_n_fk_pos = 0.001;
param.meas_n_lo_vel = 0.001;
param.meas_n_zero_vel = 0.01;
param.meas_n_om_formula = 0.01;
param.meas_n_acc_formula = 0.5;
param.meas_n_om_body = 0.001;
param.meas_n_acc_body = 0.05;
param.meas_n_yaw = 0.01;
param.meas_n_foot_height = 0.001;

%% Run EKF
tic;
[state, param] = run_ekf(resampled_data, param);
simulation_time = toc
plots;
