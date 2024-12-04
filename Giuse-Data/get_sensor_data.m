close all
clear all
clc

%%
bagFolder = 'C:\Users\Edoardo\OneDrive\Desktop\Tesi\Leonardo\Giuse-Data\edoardone_bag_2'; % create the path for the ros2 bag
bag = ros2bagreader(bagFolder); % create a ros2bagreader object

% Available Topic visualization
availableTopics = bag.AvailableTopics;
disp('Available Topic in the bag:');
disp(availableTopics);

%Define the needed topic
param.fl_imu_topic = '/imu/FL_data'; 
param.fr_imu_topic = '/imu/FR_data';
param.rl_imu_topic = '/imu/RL_data';
param.rr_imu_topic = '/imu/RR_data';
param.body_imu_topic = '/imu/trunk_data';
param.mocap_topic = '/odom/ground_truth';
param.joint_foot_topic = '/joint_states';

%% Extraction Body IMU Data
bSel = select(bag,'Topic',param.body_imu_topic);
messages = readMessages(bSel);
% initialize the variables
k = length(messages);
time=zeros(k,1);
accel_body = zeros(k,3); 
gyro_body = zeros(k,3);
for i=1:k
    time(i,:) = double(messages{i}.header.stamp.sec) + double(messages{i}.header.stamp.nanosec)*10^-9;
    accel_body(i,:) = [messages{i}.linear_acceleration.x,messages{i}.linear_acceleration.y,messages{i}.linear_acceleration.z];
    gyro_body(i,:) = [messages{i}.angular_velocity.x,messages{i}.angular_velocity.y,messages{i}.angular_velocity.z];
end
b_IMU_tinit=time(1,1);
b_IMU_tfin=time(end,1);
acc_b_IMU = timeseries([accel_body(:,1),accel_body(:,2),accel_body(:,3)],time(:,1));
acc_b_IMU.Time = acc_b_IMU.Time-acc_b_IMU.Time(1);
%acc_b_IMU.Data = movmean(acc_b_IMU.Data,5,1);
om_b_IMU = timeseries([gyro_body(:,1),gyro_body(:,2),gyro_body(:,3)],time(:,1));
om_b_IMU.Time = om_b_IMU.Time-om_b_IMU.Time(1);
%om_b_IMU.Data = movmean(om_b_IMU.Data,5,1);

%% Extraction Foot IMU Data
% FRONT LEFT FOOT

bSel = select(bag,'Topic',param.fl_imu_topic);
messages = readMessages(bSel);
% initialize the variables
k = length(messages);
time=zeros(k,1);
accel_fl = zeros(k,3); 
gyro_fl = zeros(k,3);
for i=1:k
    time(i,:) = double(messages{i}.header.stamp.sec) + double(messages{i}.header.stamp.nanosec)*10^-9;
    accel_fl(i,:) = [messages{i}.linear_acceleration.x,messages{i}.linear_acceleration.y,messages{i}.linear_acceleration.z];
    gyro_fl(i,:) = [messages{i}.angular_velocity.x,messages{i}.angular_velocity.y,messages{i}.angular_velocity.z];
end
fl_IMU_tinit=time(1,1);
fl_IMU_tfin=time(end,1);
acc_fl_IMU = timeseries([accel_fl(:,1),accel_fl(:,2),accel_fl(:,3)],time(:,1));
acc_fl_IMU.Time = acc_fl_IMU.Time-acc_fl_IMU.Time(1);
%acc_fl_IMU.Data = movmean(acc_fl_IMU.Data,5,1);
om_fl_IMU = timeseries([gyro_fl(:,1),gyro_fl(:,2),gyro_fl(:,3)],time(:,1));
om_fl_IMU.Time = om_fl_IMU.Time-om_fl_IMU.Time(1);
%om_fl_IMU.Data = movmean(om_fl_IMU.Data,5,1);

% FRONT RIGHT FOOT

bSel = select(bag,'Topic',param.fr_imu_topic);
messages = readMessages(bSel);
% initialize the variables
k = length(messages);
time=zeros(k,1);
accel_fr = zeros(k,3); 
gyro_fr = zeros(k,3);
for i=1:k
    time(i,:) = double(messages{i}.header.stamp.sec) + double(messages{i}.header.stamp.nanosec)*10^-9;
    accel_fr(i,:) = [messages{i}.linear_acceleration.x,messages{i}.linear_acceleration.y,messages{i}.linear_acceleration.z];
    gyro_fr(i,:) = [messages{i}.angular_velocity.x,messages{i}.angular_velocity.y,messages{i}.angular_velocity.z];
end
fr_IMU_tinit=time(1,1);
fr_IMU_tfin=time(end,1);
acc_fr_IMU = timeseries([accel_fr(:,1),accel_fr(:,2),accel_fr(:,3)],time(:,1));
acc_fr_IMU.Time = acc_fr_IMU.Time-acc_fr_IMU.Time(1);
%acc_fr_IMU.Data = movmean(acc_fr_IMU.Data,5,1);
om_fr_IMU = timeseries([gyro_fr(:,1),gyro_fr(:,2),gyro_fr(:,3)],time(:,1));
om_fr_IMU.Time = om_fr_IMU.Time-om_fr_IMU.Time(1);
%om_fr_IMU.Data = movmean(om_fr_IMU.Data,5,1);

% REAR RIGHT FOOT

bSel = select(bag,'Topic',param.rr_imu_topic);
messages = readMessages(bSel);
% initialize the variables
k = length(messages);
time=zeros(k,1);
accel_rr = zeros(k,3); 
gyro_rr = zeros(k,3);
for i=1:k
    time(i,:) = double(messages{i}.header.stamp.sec) + double(messages{i}.header.stamp.nanosec)*10^-9;
    accel_rr(i,:) = [messages{i}.linear_acceleration.x,messages{i}.linear_acceleration.y,messages{i}.linear_acceleration.z];
    gyro_rr(i,:) = [messages{i}.angular_velocity.x,messages{i}.angular_velocity.y,messages{i}.angular_velocity.z];
end
rr_IMU_tinit=time(1,1);
rr_IMU_tfin=time(end,1);
acc_rr_IMU = timeseries([accel_rr(:,1),accel_rr(:,2),accel_rr(:,3)],time(:,1));
acc_rr_IMU.Time = acc_rr_IMU.Time-acc_rr_IMU.Time(1);
%acc_rr_IMU.Data = movmean(acc_rr_IMU.Data,5,1);
om_rr_IMU = timeseries([gyro_rr(:,1),gyro_rr(:,2),gyro_rr(:,3)],time(:,1));
om_rr_IMU.Time = om_rr_IMU.Time-om_rr_IMU.Time(1);
%om_rr_IMU.Data = movmean(om_rr_IMU.Data,5,1);

% REAR LEFT FOOT

bSel = select(bag,'Topic',param.rl_imu_topic);
messages = readMessages(bSel);
% initialize the variables
k = length(messages);
time=zeros(k,1);
accel_rl = zeros(k,3); 
gyro_rl = zeros(k,3);
for i=1:k
    time(i,:) = double(messages{i}.header.stamp.sec) + double(messages{i}.header.stamp.nanosec)*10^-9;
    accel_rl(i,:) = [messages{i}.linear_acceleration.x,messages{i}.linear_acceleration.y,messages{i}.linear_acceleration.z];
    gyro_rl(i,:) = [messages{i}.angular_velocity.x,messages{i}.angular_velocity.y,messages{i}.angular_velocity.z];
end
rl_IMU_tinit=time(1,1);
rl_IMU_tfin=time(end,1);
acc_rl_IMU = timeseries([accel_rl(:,1),accel_rl(:,2),accel_rl(:,3)],time(:,1));
acc_rl_IMU.Time = acc_rl_IMU.Time-acc_rl_IMU.Time(1);
%acc_rl_IMU.Data = movmean(acc_rl_IMU.Data,5,1);
om_rl_IMU = timeseries([gyro_rl(:,1),gyro_rl(:,2),gyro_rl(:,3)],time(:,1));
om_rl_IMU.Time = om_rl_IMU.Time-om_rl_IMU.Time(1);
%om_rl_IMU.Data = movmean(om_rl_IMU.Data,5,1);

%% Extraction Joint Data
bSel = select(bag,'Topic',param.joint_foot_topic);
messages = readMessages(bSel);
% initialize the variables
k = length(messages);
time=zeros(k,1);
joint_vel = zeros(k,12);
joint_ang = zeros(k,12);
for i=1:k
    time(i,:) = double(messages{i}.header.stamp.sec) + double(messages{i}.header.stamp.nanosec)*10^-9;
    for j=1:12
        joint_ang(i,j) = messages{i}.position(j);
        joint_vel(i,j) = messages{i}.velocity(j);
    end
end

joint_tinit=time(1,1);
joint_tfin=time(end,1);
j_ang = timeseries([joint_ang(:,1),joint_ang(:,2),joint_ang(:,3),joint_ang(:,4),joint_ang(:,5),joint_ang(:,6),joint_ang(:,7),joint_ang(:,8),joint_ang(:,9),joint_ang(:,10),joint_ang(:,11),joint_ang(:,12)],time(:,1));
j_ang.Time = j_ang.Time-j_ang.Time(1);
% j_ang.Data = movmean(j_ang.Data,5,1);

j_vel = timeseries([joint_vel(:,1),joint_vel(:,2),joint_vel(:,3),joint_vel(:,4),joint_vel(:,5),joint_vel(:,6),joint_vel(:,7),joint_vel(:,8),joint_vel(:,9),joint_vel(:,10),joint_vel(:,11),joint_vel(:,12)],time(:,1));
j_vel.Time = j_vel.Time-j_vel.Time(1);
% j_vel.Data = movmean(j_vel.Data,5,1);