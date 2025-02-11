resampled_data = {};
resampled_data.acc_b_IMU = re_sensor_data.accel_body_IMU;
resampled_data.om_b_IMU  = re_sensor_data.gyro_body_IMU;
resampled_data.acc_fl_IMU   = re_sensor_data.accel_fl_IMU;
resampled_data.om_fl_IMU    = re_sensor_data.gyro_fl_IMU;
resampled_data.acc_fr_IMU   = re_sensor_data.accel_fr_IMU;
resampled_data.om_fr_IMU    = re_sensor_data.gyro_fr_IMU;
resampled_data.acc_rl_IMU   = re_sensor_data.accel_rl_IMU;
resampled_data.om_rl_IMU    = re_sensor_data.gyro_rl_IMU;
resampled_data.acc_rr_IMU   = re_sensor_data.accel_rr_IMU;
resampled_data.om_rr_IMU    = re_sensor_data.gyro_rr_IMU;
resampled_data.j_ang      = re_sensor_data.joint_ang;
resampled_data.j_vel      = re_sensor_data.joint_vel;
resampled_data.pos_mocap    = re_sensor_data.pos_mocap;
resampled_data.orient_mocap_euler = re_sensor_data.orient_mocap_euler;
resampled_data.foot_contact  = re_sensor_data.contact_mode;
%%
%Calculate the angular acceleration of the joints using Savitzky Golay
%Filter from velocity
[b, g] = sgolay(5, 11); 
dt = 0.005;
joint_acc_from_vel = zeros(size(resampled_data.j_vel.Data));
for p = 1:12
    joint_acc_from_vel(:,p) = conv(resampled_data.j_vel.Data(:,p), factorial(1)/(-dt^1) * g(:,2), 'same');
end
%joint_acc_from_vel = movmean(joint_acc_from_vel,5,1);
resampled_data.joint_acc = timeseries(joint_acc_from_vel, resampled_data.j_ang.Time); % as first derivative of joint velocities

%Calculate Velocity from gorund truth position
[b, g] = sgolay(5, 11); 
dt = 0.005;
mocap_vel = zeros(size(resampled_data.pos_mocap.Data));
for p = 1:3
    mocap_vel(:,p) = conv(resampled_data.pos_mocap.Data(:,p), factorial(1)/(-dt^1) * g(:,2), 'same');
end
resampled_data.vel_mocap  = mocap_vel;
%%
% Angular Velocity Comparizon
axis={'x','y','z'};
figure;
for i = 1:3
    subplot(3, 1, i);
    plot(resampled_data.acc_b_IMU.Time(200:end-100), resampled_data.om_b_b_avg(i, 200:end-100)', 'b', 'DisplayName', 'Calculated');
    hold on;
    grid on
    plot(resampled_data.acc_b_IMU.Time(200:end-100), resampled_data.om_b_IMU.Data(200:end-100,i), 'r--', 'DisplayName', 'IMU Body');
    xlabel('Time (s)');
    ylabel(['\omega_', axis{i}, ' (rad/s)']);
    legend;
    title(['Body Angular Velocity - Axis ', axis{i}]);
end

%% Linear Acceleration Comparizon
figure;
for i = 1:3
    subplot(3, 1, i);
    plot(resampled_data.acc_b_IMU.Time(200:end-100), resampled_data.a_b_b_avg(i, 200:end-100)', 'b', 'DisplayName', 'Calculated');
    hold on;
    grid on
    plot(resampled_data.acc_b_IMU.Time(200:end-100), resampled_data.acc_b_IMU.Data(200:end-100,i), 'r--', 'DisplayName', 'IMU Body');
    xlabel('Time (s)');
    ylabel(['a_', axis{i}, ' (m/s^2)']);
    legend;
    title(['Body Linear Acceleration - Axis ', axis{i}]);
end


%% Noise Introduction
% Introduce high noise to body IMU data
large_noise_value = 50; % Adjust as needed
resampled_data.acc_b_IMU.Data = resampled_data.acc_b_IMU.Data + randn(size(resampled_data.acc_b_IMU.Data)) * large_noise_value;%re_sensor_data.gyro_body_IMU.Data-re_sensor_data.gyro_body_IMU.Data;re_sensor_data.gyro_body_IMU.Data + randn(size(re_sensor_data.gyro_body_IMU.Data)) * large_noise_value;
resampled_data.om_b_IMU.Data = resampled_data.om_b_IMU.Data + randn(size(resampled_data.om_b_IMU.Data)) * large_noise_value;%re_sensor_data.gyro_body_IMU.Data-re_sensor_data.gyro_body_IMU.Data;

