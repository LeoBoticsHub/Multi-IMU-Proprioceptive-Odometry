% Vedere il paragone tra due metodi di calcolo per le velocità angolari e
% le accelerazioni angolari. Sfruttando la derivata prima o seconda dei
% joint angle o sfruttando la velocità angolare dei joint o la sua derivata

% --- Plot Accelerazioni per ciascun joint ---
figure;
for p = 1:12
    subplot(4,3,p);
    plot(resampled_data.j_ang.Time(:), joint_acc_smooth_data(:,p), 'b', 'DisplayName', 'From Angle (SG)');
    hold on;
    plot(resampled_data.j_ang.Time(:), joint_acc_from_vel(:,p), 'r', 'DisplayName', 'From Velocity');
    xlabel('Time [s]');
    ylabel('Acceleration [rad/s^2]');
    
    % Determina la gamba in base a p
    if p >=1 && p<=3
        leg_name = 'FL';
    elseif p>=4 && p<=6
        leg_name = 'FR';
    elseif p>=7 && p<=9
        leg_name = 'RR';
    else
        leg_name = 'RL';
    end
    
    title([leg_name, ' Joint ', num2str(mod(p-1,3)+1), ' Acceleration Comparison']);
    legend('Location','best');
end

% --- Plot Velocità per ciascun joint ---
figure;
for p = 1:12
    subplot(4,3,p);
    plot(resampled_data.j_ang.Time(:), joint_vel_smooth_data(:,p), 'b', 'DisplayName', 'From Angle (SG)');
    hold on;
    plot(resampled_data.j_ang.Time(:), resampled_data.j_vel.Data(:,p), 'r', 'DisplayName', 'Encoders');
    xlabel('Time [s]');
    ylabel('Velocity [rad/s]');
    
    % Determina la gamba in base a p
    if p >=1 && p<=3
        leg_name = 'FL';
    elseif p>=4 && p<=6
        leg_name = 'FR';
    elseif p>=7 && p<=9
        leg_name = 'RR';
    else
        leg_name = 'RL';
    end
    
    title([leg_name, ' Joint ', num2str(mod(p-1,3)+1), ' Velocity Comparison']);
    legend('Location','best');
end

%% Comparizon betweeen the use of Savitzky Golay Filter and Gradient matlab fucntion for the computation of the derivatives
%% Calcolo Velocità e Accelerazioni con metodo gradient

for p = 1:12
    grad_vel_from_ang(:,p) = gradient(resampled_data.j_ang.Data(:,p), dt);
    grad_acc_from_ang(:,p) = gradient(grad_vel_from_ang(:,p), dt);
    grad_acc_from_vel(:,p) = gradient(resampled_data.j_vel.Data(:,p), dt);
end

% --- Plot Accelerazioni per ciascun joint ---
figure;
for p = 1:12
    subplot(4,3,p);
    plot(resampled_data.j_ang.Time(:), grad_acc_from_vel(:,p), 'b', 'DisplayName', 'From Velocity with Gradient');
    hold on;
    plot(resampled_data.j_ang.Time(:), joint_acc_from_vel(:,p), 'r', 'DisplayName', 'From Velocity with SG');
    xlabel('Time [s]');
    ylabel('Acceleration [rad/s^2]');
    
    % Determina la gamba in base a p
    if p >=1 && p<=3
        leg_name = 'FL';
    elseif p>=4 && p<=6
        leg_name = 'FR';
    elseif p>=7 && p<=9
        leg_name = 'RR';
    else
        leg_name = 'RL';
    end
    
    title([leg_name, ' Joint ', num2str(mod(p-1,3)+1), ' Acceleration Comparison']);
    legend('Location','best');
end

% --- Plot Velocità per ciascun joint ---
figure;
for p = 1:12
    subplot(4,3,p);
    plot(resampled_data.j_ang.Time(:), grad_vel_from_ang(:,p), 'b', 'DisplayName', 'From Angle Gradient');
    hold on;
    plot(resampled_data.j_ang.Time(:), resampled_data.j_vel.Data(:,p), 'r', 'DisplayName', 'Encoders');
    xlabel('Time [s]');
    ylabel('Velocity [rad/s]');
    
    % Determina la gamba in base a p
    if p >=1 && p<=3
        leg_name = 'FL';
    elseif p>=4 && p<=6
        leg_name = 'FR';
    elseif p>=7 && p<=9
        leg_name = 'RR';
    else
        leg_name = 'RL';
    end
    
    title([leg_name, ' Joint ', num2str(mod(p-1,3)+1), ' Velocity Comparison']);
    legend('Location','best');
end
