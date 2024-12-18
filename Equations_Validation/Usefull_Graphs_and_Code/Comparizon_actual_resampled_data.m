% This code is used to visualize the difference between the actual data coming from 
% the Gazebo simulation and the resampled data

joint = {'Hip','Thigh','Calf'};
axis = {'x','y','z'};
%% Body IMU linear acceleration graph
acc_b_IMU.Time=acc_b_IMU.Time-acc_b_IMU.Time(1);
figure(1);
for i = 1:3
    subplot(3, 1, i);
    plot(acc_b_IMU.Time, acc_b_IMU.Data(:, i), 'b', 'DisplayName', 'Original');
    hold on;
    plot(resampled_data.acc_b_IMU.Time, resampled_data.acc_b_IMU.Data(:, i), 'r', 'DisplayName', 'Resampled');
    xlabel('Time (s)');
    ylabel(['a_', axis{i}, ' (m/s^2)']);
    legend;
    title(['Original VS Resampled data (Body IMU linear acceleration) - Axis ', axis{i}]);
end
%% Body IMU angular velocity graph
om_b_IMU.Time=om_b_IMU.Time-om_b_IMU.Time(1);
figure(2);
for i = 1:3
    subplot(3, 1, i);
    plot(om_b_IMU.Time, om_b_IMU.Data(:, i), 'b', 'DisplayName', 'Original');
    hold on;
    plot(resampled_data.om_b_IMU.Time, resampled_data.om_b_IMU.Data(:, i), 'r', 'DisplayName', 'Resampled');
    xlabel('Time (s)');
    ylabel(['\omega_', axis{i}, ' (rad/s)']);
    legend;
    title(['Original VS Resampled data (Body IMU Angular velocity) - Axis ', axis{i}]);
end
%% Joint Angles Graph
j_ang.Time=j_ang.Time-j_ang.Time(1)-(start_time_list(1)-start_time_list(11));
figure(3);
for i = 1:3 %plotting only the joint of the first leg
    subplot(3, 1, i);
    plot(j_ang.Time, j_ang.Data(:, i), 'b', 'DisplayName', 'Original');
    hold on;
    plot(resampled_data.j_ang.Time, resampled_data.j_ang.Data(:, i), 'r', 'DisplayName', 'Resampled');
    xlabel('Time (s)');
    ylabel(['\phi_', num2str(i), ' (rad)']);
    legend;
    title(['Original VS Resampled data (Joint angle) - ', joint{i}]);
end
%% Joint Angular Velocities Graph
j_vel.Time=j_vel.Time-j_vel.Time(1)-(start_time_list(1)-start_time_list(11));
figure(4);
for i = 1:3 %plotting only the joint velocities of the first leg
    subplot(3, 1, i);
    plot(j_vel.Time, j_vel.Data(:, i), 'b', 'DisplayName', 'Original');
    hold on;
    plot(resampled_data.j_vel.Time, resampled_data.j_vel.Data(:, i), 'r', 'DisplayName', 'Resampled');
    xlabel('Time (s)');
    ylabel(['d\phi_', num2str(i), ' (rad/s)']);
    legend;
    title(['Original VS Resampled data (Joint angular velocity) - ', joint{i}]);
end
%% Foot IMU linear acceleration graph
acc_fr_IMU.Time=acc_fr_IMU.Time-acc_fr_IMU.Time(1);
figure(5);
for i = 1:3
    subplot(3, 1, i);
    plot(acc_fr_IMU.Time, acc_fr_IMU.Data(:, i), 'b', 'DisplayName', 'Original');
    hold on;
    plot(resampled_data.acc_fr_IMU.Time, resampled_data.acc_fr_IMU.Data(:, i), 'r', 'DisplayName', 'Resampled');
    xlabel('Time (s)');
    ylabel(['a_', axis{i}, ' (m/s^2)']);
    legend;
    title(['Original VS Resampled data (Foot IMU (fr) Linear Acceleration) - Axis ', axis{i}]);
end
%% Foot IMU angular velocity graph
om_fr_IMU.Time=om_fr_IMU.Time-om_fr_IMU.Time(1);
figure(6);
for i = 1:3
    subplot(3, 1, i);
    plot(om_fr_IMU.Time, om_fr_IMU.Data(:, i), 'b', 'DisplayName', 'Original');
    hold on;
    plot(resampled_data.om_fr_IMU.Time, resampled_data.om_fr_IMU.Data(:, i), 'r', 'DisplayName', 'Resampled');
    xlabel('Time (s)');
    ylabel(['\omega_', axis{i}, ' (rad/s)']);
    legend;
    title(['Original VS Resampled data (Foot IMU (fr) Angular Velocity) - Axis ', axis{i}]);
end