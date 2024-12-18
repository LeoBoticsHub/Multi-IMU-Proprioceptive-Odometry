% This code is used to plot the joint angles and joint velocities

%% Joint Angles for each leg
colors = {'b', 'r', 'g', 'k'}; % Blu, Rosso, Verde, Nero per le gambe FL, FR, RL, RR
name={'hip','thigh','calf'};

figure(20)
for i = 1:3 % Per ogni componente x, y, z
    subplot(3, 1, i);
    for leg_id=1:4
    hold on;
    plot(resampled_data.j_ang.Time, resampled_data.j_ang.Data(:,(leg_id-1)*3+i), colors{leg_id}, 'DisplayName', ['Leg ' num2str(leg_id)]);
    grid on
    xlabel('Time (s)');
    ylabel(['\phi_', num2str(i), ' (rad)']);
    legend('show');
    title(['Joint Angle - ', name{i}]);
    end
end

%% Joint Angular Velocities for each leg
figure(21)
for i = 1:3 % Per ogni componente x, y, z
    subplot(3, 1, i);
    for leg_id=1:4
    hold on;
    plot(resampled_data.j_vel.Time, resampled_data.j_vel.Data(:,(leg_id-1)*3+i), colors{leg_id}, 'DisplayName', ['Leg ' num2str(leg_id)]);
    grid on
    xlabel('Tempo (s)');
    ylabel(['d\phi_', num2str(i), ' (rad/s)']);
    legend('show');
    title(['Joint Angular Velocity - ', name{i}]);
    end
end
