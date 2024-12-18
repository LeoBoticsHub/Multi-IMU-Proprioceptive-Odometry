close all
clear all
clc

%%
bagFolder = 'C:\Users\Edoardo\OneDrive\Desktop\Tesi\Leonardo\Ros2\edoardone_bag_trot'; % create the path for the ros2 bag
bag = ros2bagreader(bagFolder); % create a ros2bagreader object

% Available Topic visualization
availableTopics = bag.AvailableTopics;
disp('Available Topic in the bag:');
disp(availableTopics);

% Define the needed topic
param.fl_imu_topic = '/imu/FL_data'; 
param.fr_imu_topic = '/imu/FR_data';
param.rl_imu_topic = '/imu/RL_data';
param.rr_imu_topic = '/imu/RR_data';
param.body_imu_topic = '/imu/trunk_data';
param.mocap_topic = '/odom/ground_truth';
param.joint_foot_topic = '/joint_group_effort_controller/joint_trajectory';
param.joint_readings = '/joint_states';

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

acc_b_IMU = timeseries([accel_body(:,1),accel_body(:,2),accel_body(:,3)],time(:,1));
%acc_b_IMU.Time = acc_b_IMU.Time-acc_b_IMU.Time(1);
%acc_b_IMU.Data = movmean(acc_b_IMU.Data,5,1);
om_b_IMU = timeseries([gyro_body(:,1),gyro_body(:,2),gyro_body(:,3)],time(:,1));
%om_b_IMU.Time = om_b_IMU.Time-om_b_IMU.Time(1);
%om_b_IMU.Data = movmean(om_b_IMU.Data,5,1);

%Check on time
stop=0;
dt_list = zeros(k-1,1);
dt_list = diff(time);%time(i) - time(i-1);
if (any(dt_list<=0) && stop==0)
    fprintf('dt_list is NOT strictly higher then zero \n');
    stop=1;
end
dt1=mean(dt_list);
disp('BODY IMU Simulation:')
fprintf('T_start= %f s\n', time(1));
fprintf('T_end= %f s\n', time(end));
fprintf('T_tot= %f s\n', time(end)-time(1));
fprintf('dt_mean= %f s\n', dt1);
fprintf('dt_max= %f s\n', max(dt_list));
fprintf('dt_min= %f s\n\n', min(dt_list));

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
acc_fl_IMU = timeseries([accel_fl(:,1),accel_fl(:,2),accel_fl(:,3)],time(:,1));
%acc_fl_IMU.Time = acc_fl_IMU.Time-acc_fl_IMU.Time(1);
%acc_fl_IMU.Data = movmean(acc_fl_IMU.Data,5,1);
om_fl_IMU = timeseries([gyro_fl(:,1),gyro_fl(:,2),gyro_fl(:,3)],time(:,1));
%om_fl_IMU.Time = om_fl_IMU.Time-om_fl_IMU.Time(1);
%om_fl_IMU.Data = movmean(om_fl_IMU.Data,5,1);

%Check on time
stop=0;
dt_list = zeros(k-1,1);
dt_list = diff(time);%time(i) - time(i-1);
if (any(dt_list<=0) && stop==0)
    fprintf('dt_list is NOT strictly higher then zero \n');
    stop=1;
end
dt2=mean(dt_list);
disp('FL_IMU Simulation:')
fprintf('T_start= %f s\n', time(1));
fprintf('T_end= %f s\n', time(end));
fprintf('T_tot= %f s\n', time(end)-time(1));
fprintf('dt_mean= %f s\n', dt2);
fprintf('dt_max= %f s\n', max(dt_list));
fprintf('dt_min= %f s\n\n', min(dt_list));

%% FRONT RIGHT FOOT
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
acc_fr_IMU = timeseries([accel_fr(:,1),accel_fr(:,2),accel_fr(:,3)],time(:,1));
%acc_fr_IMU.Time = acc_fr_IMU.Time-acc_fr_IMU.Time(1);
%acc_fr_IMU.Data = movmean(acc_fr_IMU.Data,5,1);
om_fr_IMU = timeseries([gyro_fr(:,1),gyro_fr(:,2),gyro_fr(:,3)],time(:,1));
%om_fr_IMU.Time = om_fr_IMU.Time-om_fr_IMU.Time(1);
%om_fr_IMU.Data = movmean(om_fr_IMU.Data,5,1);

%Check on time
stop=0;
dt_list = zeros(k-1,1);
dt_list = diff(time);%time(i) - time(i-1);
if (any(dt_list<=0) && stop==0)
    fprintf('dt_list is NOT strictly higher then zero \n');
    stop=1;
end
dt3=mean(dt_list);
disp('FR_IMU Simulation:')
fprintf('T_start= %f s\n', time(1));
fprintf('T_end= %f s\n', time(end));
fprintf('T_tot= %f s\n', time(end)-time(1));
fprintf('dt_mean= %f s\n', dt3);
fprintf('dt_max= %f s\n', max(dt_list));
fprintf('dt_min= %f s\n\n', min(dt_list));

%% REAR LEFT FOOT
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
acc_rl_IMU = timeseries([accel_rl(:,1),accel_rl(:,2),accel_rl(:,3)],time(:,1));
%acc_rl_IMU.Time = acc_rl_IMU.Time-acc_rl_IMU.Time(1);
%acc_rl_IMU.Data = movmean(acc_rl_IMU.Data,5,1);
om_rl_IMU = timeseries([gyro_rl(:,1),gyro_rl(:,2),gyro_rl(:,3)],time(:,1));
%om_rl_IMU.Time = om_rl_IMU.Time-om_rl_IMU.Time(1);
%om_rl_IMU.Data = movmean(om_rl_IMU.Data,5,1);

%Check on time
stop=0;
dt_list = zeros(k-1,1);
dt_list = diff(time);%time(i) - time(i-1);
if (any(dt_list<=0) && stop==0)
    fprintf('dt_list is NOT strictly higher then zero \n');
    stop=1;
end
dt5=mean(dt_list);
disp('RL_IMU Simulation:')
fprintf('T_start= %f s\n', time(1));
fprintf('T_end= %f s\n', time(end));
fprintf('T_tot= %f s\n', time(end)-time(1));
fprintf('dt_mean= %f s\n', dt5);
fprintf('dt_max= %f s\n', max(dt_list));
fprintf('dt_min= %f s\n\n', min(dt_list));

%% REAR RIGHT FOOT
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
acc_rr_IMU = timeseries([accel_rr(:,1),accel_rr(:,2),accel_rr(:,3)],time(:,1));
%acc_rr_IMU.Time = acc_rr_IMU.Time-acc_rr_IMU.Time(1);
%acc_rr_IMU.Data = movmean(acc_rr_IMU.Data,5,1);
om_rr_IMU = timeseries([gyro_rr(:,1),gyro_rr(:,2),gyro_rr(:,3)],time(:,1));
%om_rr_IMU.Time = om_rr_IMU.Time-om_rr_IMU.Time(1);
%om_rr_IMU.Data = movmean(om_rr_IMU.Data,5,1);

%Check on time
dt_list = zeros(k-1,1);
dt_list = diff(time);%time(i) - time(i-1);
if (any(dt_list<=0) && stop==0)
    fprintf('dt_list is NOT strictly higher then zero \n');
    stop=1;
end
dt4=mean(dt_list);
disp('RR_IMU Simulation:')
fprintf('T_start= %f s\n', time(1));
fprintf('T_end= %f s\n', time(end));
fprintf('T_tot= %f s\n', time(end)-time(1));
fprintf('dt_mean= %f s\n', dt4);
fprintf('dt_max= %f s\n', max(dt_list));
fprintf('dt_min= %f s\n\n', min(dt_list));

%% Extraction Joint Data 
bSel = select(bag,'Topic',param.joint_readings);
messages = readMessages(bSel);
stop=0;
% Liste dinamiche (inizialmente vuote) da riempire solo se ci sono velocità
time_list = [];
joint_ang_list = [];
joint_vel_list = [];

if isfield(messages{i}, 'name') %we use the joint readings
    % Joint order definition
    joint_order = { ...
        'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', ...
        'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', ...
        'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint', ...
        'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'};
     for i=1:length(messages)
        % Verifica se esistono dati di velocity e non sono vuoti
        if isfield(messages{i}, 'velocity') && ~isempty(messages{i}.velocity)
            
            % Leggi il timestamp
            current_time = double(messages{i}.header.stamp.sec) + double(messages{i}.header.stamp.nanosec)*10^-9;
    
            % Ottieni i nomi dei giunti, posizioni e velocità
            current_names = messages{i}.name;
            current_positions = messages{i}.position;
            current_velocities = messages{i}.velocity;
    
            % Inizializza i vettori per angoli e velocità per questo messaggio
            current_jang = NaN(1,12);
            current_jvel = NaN(1,12);
    
            % Popola i vettori con l'ordine desiderato
            for j=1:length(joint_order)
                idx = find(strcmp(current_names, joint_order{j}));
                if ~isempty(idx)
                    current_jang(j) = current_positions(idx);
                    current_jvel(j) = current_velocities(idx);
                else
                    % Se non trovo quel giunto, assegno NaN e continuo
                    current_jang(j) = NaN;
                    current_jvel(j) = NaN;
                    warning('Giunto %s non trovato nel messaggio %d', joint_order{j}, i);
                end
            end
    
            % Aggiungo i dati alle liste
            time_list = [time_list; current_time];
            joint_ang_list = [joint_ang_list; current_jang];
            joint_vel_list = [joint_vel_list; current_jvel];
        else
            % Nessun dato di velocità per questo messaggio, si ignora
        end
    end
    
    % Creazione delle timeseries solo con i dati validi
    j_ang = timeseries(joint_ang_list, time_list);
    j_vel = timeseries(joint_vel_list, time_list);
    
    % Controlli sul tempo
    if length(time_list) > 1
        dt_list = diff(time_list);
        if any(dt_list <= 0)
            fprintf('dt_list non è strettamente maggiore di zero\n');
        end
        dt6 = mean(dt_list);
        disp('JOINT Simulation:')
        fprintf('T_start= %f s\n', time_list(1));
        fprintf('T_end= %f s\n', time_list(end));
        fprintf('T_tot= %f s\n', time_list(end)-time_list(1));
        fprintf('dt_mean= %f s\n', dt6);
        fprintf('dt_max= %f s\n', max(dt_list));
        fprintf('dt_min= %f s\n\n', min(dt_list));
    else
        disp('Nessun dato valido (con velocità) trovato.');
    end
else %we use the joint trajectories
    bSel = select(bag,'Topic',param.joint_foot_topic);
    messages = readMessages(bSel);
    % initialize the variables
    k = length(messages);
    time=zeros(k,1);
    joint_vel = zeros(k,12);
    joint_ang = zeros(k,12);
    dt_list = zeros(k,1);
    for i=1:k
        time(i,:) = double(messages{i}.header.stamp.sec) + double(messages{i}.header.stamp.nanosec)*10^-9;
        for j=1:12
            joint_ang(i,j) = messages{i}.points.positions(j);
            joint_vel(i,j) = messages{i}.points.velocities(j);
        end
    end
    
    j_ang = timeseries([joint_ang(:,1),joint_ang(:,2),joint_ang(:,3),joint_ang(:,4),joint_ang(:,5),joint_ang(:,6),joint_ang(:,7),joint_ang(:,8),joint_ang(:,9),joint_ang(:,10),joint_ang(:,11),joint_ang(:,12)],time(:,1));
    %j_ang.Time = j_ang.Time-j_ang.Time(1);
    % j_ang.Data = movmean(j_ang.Data,5,1);
    
    j_vel = timeseries([joint_vel(:,1),joint_vel(:,2),joint_vel(:,3),joint_vel(:,4),joint_vel(:,5),joint_vel(:,6),joint_vel(:,7),joint_vel(:,8),joint_vel(:,9),joint_vel(:,10),joint_vel(:,11),joint_vel(:,12)],time(:,1));
    %j_vel.Time = j_vel.Time-j_vel.Time(1);
    % j_vel.Data = movmean(j_vel.Data,5,1);
    
    %Check on time
    stop=0;
    dt_list = zeros(k-1,1);
    dt_list = diff(time);%time(i) - time(i-1);
    if (any(dt_list<=0) && stop==0)
        fprintf('dt_list is NOT strictly higher then zero \n');
        stop=1;
    end
    dt6=mean(dt_list);
    disp('JOINT Simulation:')
    fprintf('T_start= %f s\n', time(1));
    fprintf('T_end= %f s\n', time(end));
    fprintf('T_tot= %f s\n', time(end)-time(1));
    fprintf('dt_mean= %f s\n', dt6);
    fprintf('dt_max= %f s\n', max(dt_list));
    fprintf('dt_min= %f s\n\n', min(dt_list));
end

if stop==1
    [dt6,j_ang,j_vel]=unique_points(dt_list,j_ang,j_vel)
end


%% RESAMPLE DATA
start_time_list = [acc_b_IMU.Time(1);
                 om_b_IMU.Time(1);
                 acc_fl_IMU.Time(1);
                 om_fl_IMU.Time(1);
                 acc_fr_IMU.Time(1);
                 om_fr_IMU.Time(1);
                 acc_rl_IMU.Time(1);
                 om_rl_IMU.Time(1);
                 acc_rr_IMU.Time(1);
                 om_rr_IMU.Time(1);
                 j_ang.Time(1);
                 j_vel.Time(1)];

end_time_list = [acc_b_IMU.Time(end);
                 om_b_IMU.Time(end);
                 acc_fl_IMU.Time(end);
                 om_fl_IMU.Time(end);
                 acc_fr_IMU.Time(end);
                 om_fr_IMU.Time(end);
                 acc_rl_IMU.Time(end);
                 om_rl_IMU.Time(end);
                 acc_rr_IMU.Time(end);
                 om_rr_IMU.Time(end);
                 j_ang.Time(end);
                 j_vel.Time(end)];
max_start_time = max(start_time_list);
min_end_time = min(end_time_list);
dt=min([dt1,dt2,dt3,dt4,dt5,dt6]);
common_time_vector = max_start_time : dt : min_end_time;
resampled_data = {};
% % Linear interpolation method
% resampled_data.acc_b_IMU = resample(acc_b_IMU, common_time_vector);
% resampled_data.om_b_IMU  = resample(om_b_IMU, common_time_vector);
% resampled_data.acc_fl_IMU   = resample(acc_fl_IMU, common_time_vector);
% resampled_data.om_fl_IMU    = resample(om_fl_IMU, common_time_vector);
% resampled_data.acc_fr_IMU   = resample(acc_fr_IMU, common_time_vector);
% resampled_data.om_fr_IMU    = resample(om_fr_IMU, common_time_vector);
% resampled_data.acc_rl_IMU   = resample(acc_rl_IMU, common_time_vector);
% resampled_data.om_rl_IMU    = resample(om_rl_IMU, common_time_vector);
% resampled_data.acc_rr_IMU   = resample(acc_rr_IMU, common_time_vector);
% resampled_data.om_rr_IMU    = resample(om_rr_IMU, common_time_vector);
% resampled_data.j_ang      = resample(j_ang, common_time_vector);
% resampled_data.j_vel      = resample(j_vel, common_time_vector);


% Other interpolation Method (pchip)
% Need to use unique_points function if dt_list is not strictly increasing
original_ts_names = {'acc_b_IMU', 'om_b_IMU', 'acc_fl_IMU', 'om_fl_IMU', ...
                     'acc_fr_IMU', 'om_fr_IMU', 'acc_rl_IMU', 'om_rl_IMU', ...
                     'acc_rr_IMU', 'om_rr_IMU', 'j_ang', 'j_vel'};
for i = 1:length(original_ts_names)
    ts_name = original_ts_names{i};
    ts = eval(ts_name); % Get the original timeseries
    % Interpolation
    resampled_data_values = interp1(ts.Time, ts.Data, common_time_vector, 'pchip');
    % Create a new timeseries
    resampled_data.(ts_name) = timeseries(resampled_data_values, common_time_vector);
end

% Subtraction of initial instant to Time vector
fields = fieldnames(resampled_data);
for i = 1:length(fields)
    resampled_data.(fields{i}).Time = resampled_data.(fields{i}).Time - resampled_data.(fields{i}).Time(1);
end

%%
% Calculate the angular velocity of the joints using Savitzky Golay Filter from angle 
[b,g] = sgolay(5,11); 
joint_vel_smooth_data = zeros(length(resampled_data.j_ang.Data),12);
for p = 1:12
   joint_vel_smooth_data(:,p) = conv(resampled_data.j_ang.Data(:,p), factorial(1)/(-dt)^1 * g(:,2), 'same');
end
%joint_vel_smooth_data = movmean(joint_vel_smooth_data,5,1);
resampled_data.joint_vel = timeseries(joint_vel_smooth_data,resampled_data.j_ang.Time); % as first derivative of joint angles

%Calculate the angular acceleration of the joints using Savitzky Golay Filter from angle
[b, g] = sgolay(5, 11); 
joint_acc_smooth_data = zeros(length(resampled_data.j_ang.Data), 12);
for p = 1:12
    joint_acc_smooth_data(:, p) = conv(resampled_data.j_ang.Data(:,p), factorial(2)/(-dt)^2 * g(:, 3), 'same');
end
%joint_acc_smooth_data = movmean(joint_acc_smooth_data,5,1);
resampled_data.j_acc = timeseries(joint_acc_smooth_data, resampled_data.j_ang.Time); % as second derivative of joint angles

%Calculate the angular acceleration of the joints using Savitzky Golay
%Filter from velocity
joint_acc_from_vel = zeros(size(resampled_data.j_vel.Data));
for p = 1:12
    joint_acc_from_vel(:,p) = conv(resampled_data.j_vel.Data(:,p), factorial(1)/(-dt^1) * g(:,2), 'same');
end
%joint_acc_from_vel = movmean(joint_acc_from_vel,5,1);
resampled_data.joint_acc = timeseries(joint_acc_from_vel, resampled_data.j_ang.Time); % as first derivative of joint velocities

