
colors = {'b', 'r', 'g', 'k'}; % Blue, Red, Green, black for legs FL, FR, RL, RR
time_vector = resampled_data.om_b_IMU.Time(init_idx:end_idx);
axis = {'x','y','z'};

%% Angular velocity comparizon for each leg and ground truth
figure;

for i = 1:3 
    subplot(3, 1, i);
    hold on;
    grid on
    for leg_id = 1:4
        plot(time_vector, om_b_b(i, :, leg_id), colors{leg_id}, 'DisplayName', ['Leg ' num2str(leg_id)]);
    end
    plot(time_vector, w(i, init_idx:end_idx), 'm--', 'LineWidth', 1, 'DisplayName', 'IMU Body');
    
    xlabel('Time (s)');
    ylabel(['\omega_', axis{i}, ' (rad/s)']);
    legend('show');
    title(['Body Angular Velocity - Axis ', axis{i}]);
end

%% Linear acceleration comparizon for each leg and ground truth
figure;
for i = 1:3 
    subplot(3, 1, i);
    hold on;
    grid on
    for leg_id = 1:4 
        plot(time_vector, squeeze(a_b_b(i, :, leg_id)), colors{leg_id}, 'DisplayName', ['Leg ' num2str(leg_id)]);
    end
    plot(time_vector, a(i, init_idx:end_idx), 'm--', 'LineWidth', 1, 'DisplayName', 'IMU Body');
    xlabel('Time (s)');
    ylabel(['a_', axis{i}, ' (m/s^2)']);
    legend('show');
    title(['Body Linear Acceleration - Axis ', axis{i}]);
end