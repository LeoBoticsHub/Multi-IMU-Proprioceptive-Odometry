% The provided code configures a series of parameters within a structure called param, 
% representing the geometric and operational characteristics of the legs of the Go1 quadruped robot.
%% put some necessary variables in param struct
param.num_leg = 4; % number of legs
param.leg_name = ['FL','FR','RL', 'RR']; % leg name
param.all_leg = [1,2,3,4]; % leg identification number

% offset wth respect to robot body center
param.ox = [0.1881,0.1881,-0.1881,-0.1881]; % longitudinal axis 
param.oy = [0.04675,-0.04675,0.04675,-0.04675]; % lateral axis (positive on the left side)
param.d = [0.08,-0.08,0.08,-0.08]; % lateral offset between hip and thigh
param.lt = 0.213; % thigh length
param.lc = 0.213; % calf length

param.leg = zeros(4,4);
param.leg(:,1) = [param.ox(1);param.oy(1);param.d(1);param.lt];
param.leg(:,2) = [param.ox(2);param.oy(2);param.d(2);param.lt];
param.leg(:,3) = [param.ox(3);param.oy(3);param.d(3);param.lt];
param.leg(:,4) = [param.ox(4);param.oy(4);param.d(4);param.lt];

param.R_fs = {[1 0 0; %Rotation matrix from IMU frame (s:sensor) to foot frame for each leg
               0 1 0;
               0 0 1], 
             [1 0 0;
               0 1 0;
               0 0 1],
             [1 0 0;
               0 1 0;
               0 0 1],
             [1 0 0;
               0 1 0;
               0 0 1]};