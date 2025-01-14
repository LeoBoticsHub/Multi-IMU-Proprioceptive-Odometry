% plot the leg variables considering highlithing only the contribution when the leg is in
% contact with the ground.

% PS remember to run first get_sensor_data and then only use_formulas

colors = {'b', 'r', 'g', 'k'}; % Blue, Red, Green, black for legs FL, FR, RL, RR
time_vector = resampled_data.om_b_IMU.Time;
leg_fields = {'om_fl_IMU','om_fr_IMU','om_rl_IMU','om_rr_IMU'}; % For data access
axis_names = {'x','y','z'};
leg_names = {'FL','FR','RL','RR'};   % For display
colors_stance = 'g';  % Green for stance
colors_swing  = 'b';  % Blue  for swing
%% Extract foot contact data
% Make sure foot_contact is aligned in time with om_b_b, a_b_b, etc.
foot_contact = resampled_data.foot_contact.Data; 
% foot_contact is [num_samples x 4]
%   Columns: 1:FL, 2:FR, 3:RL, 4:RR
%   Values > 0.5 => Contact, < 0.5 => Swing

%% Angular velocity comparison (stance only)
figure;
for i = 1:3
    subplot(3, 1, i);
    hold on; grid on;
    for leg_id = 1:4
        % Extract the original angular velocity for the current leg & axis
        ang_vel_leg = squeeze(om_b_b(i, :, leg_id));  % 1 x num_samples

        % "Mask out" swing phases by setting those samples to NaN
        %   (foot_contact(:,leg_id) < 0.5 => swing)
        idx_swing = foot_contact(:, leg_id) < 0.5; 
        ang_vel_leg(idx_swing) = NaN;  % cancel out swings

        plot(time_vector, ang_vel_leg, colors{leg_id}, ...
             'DisplayName', ['Leg ' num2str(leg_id) ' (stance)']);
    end
    
    % Plot the IMU Body reference
    plot(time_vector, w(i, init_idx:end_idx), 'm--', 'LineWidth', 1, ...
         'DisplayName', 'IMU Body');
    
    xlabel('Time (s)');
    ylabel(['\omega_', axis_names{i}, ' (rad/s)']);
    legend('show');
    title(['Body Angular Velocity - Axis ', axis_names{i}]);
end


%% Linear acceleration comparison (stance only)
figure;
for i = 1:3
    subplot(3, 1, i);
    hold on; grid on;
    for leg_id = 1:4
        % Extract the original linear acceleration for the current leg & axis
        lin_acc_leg = squeeze(a_b_b(i, :, leg_id));  % 1 x num_samples

        % "Mask out" swing phases
        idx_swing = foot_contact(:, leg_id) < 0.5;
        lin_acc_leg(idx_swing) = NaN;

        plot(time_vector, lin_acc_leg, colors{leg_id}, ...
             'DisplayName', ['Leg ' num2str(leg_id) ' (stance)']);
    end

    % Plot the IMU Body reference
    plot(time_vector, a(i, init_idx:end_idx), 'm--', 'LineWidth', 1, ...
         'DisplayName', 'IMU Body');
    
    xlabel('Time (s)');
    ylabel(['a_', axis_names{i}, ' (m/s^2)']);
    legend('show');
    title(['Body Linear Acceleration - Axis ', axis_names{i}]);
end

%% Plot the angular velocity coming from all the IMU sensors in their reference frame only stance phase

figure;
for i = 1:3
    subplot(3,1,i); 
    hold on; grid on;

    % --- Front Left (FL) ---
    ang_vel_fl = resampled_data.om_fl_IMU.Data(:,i);
    % Mask out swing phase (where contact < 0.5)
    ang_vel_fl(resampled_data.foot_contact.Data(:,1) < 0.5) = NaN;
    plot(time_vector, ang_vel_fl, colors{1}, 'DisplayName','Leg 1 (FL, stance)');

    % --- Front Right (FR) ---
    ang_vel_fr = resampled_data.om_fr_IMU.Data(:,i);
    ang_vel_fr(resampled_data.foot_contact.Data(:,2) < 0.5) = NaN;
    plot(time_vector, ang_vel_fr, colors{2}, 'DisplayName','Leg 2 (FR, stance)');

    % --- Rear Left (RL) ---
    ang_vel_rl = resampled_data.om_rl_IMU.Data(:,i);
    ang_vel_rl(resampled_data.foot_contact.Data(:,3) < 0.5) = NaN;
    plot(time_vector, ang_vel_rl, colors{3}, 'DisplayName','Leg 3 (RL, stance)');

    % --- Rear Right (RR) ---
    ang_vel_rr = resampled_data.om_rr_IMU.Data(:,i);
    ang_vel_rr(resampled_data.foot_contact.Data(:,4) < 0.5) = NaN;
    plot(time_vector, ang_vel_rr, colors{4}, 'DisplayName','Leg 4 (RR, stance)');

    % Labels, Legend, etc.
    xlabel('Time (s)');
    ylabel(['\omega_', axis_names{i}, ' (rad/s)']);
    legend('show');
    title(['Foot IMU Angular Velocity (stance only) — axis: ', axis_names{i}]);
end

%% Plot the angular velocity coming from the IMU sensors in their reference frame both stance and swing phase
leg_id=1; % leg: 1:FL, 2:FR, 3:RL, 4:RR
figure('Name', ['Leg ', num2str(leg_id), ' - ', leg_names{leg_id}]);
% For each axis (x,y,z) => 3 subplots
for i = 1:3
    subplot(3,1,i);
    hold on;  grid on;
    % Create stance vs swing signals
    w_stance = resampled_data.(leg_fields{leg_id}).Data(:, leg_id);
    w_swing  = resampled_data.(leg_fields{leg_id}).Data(:, leg_id);
    % Stance: keep data where contact >= 0.5, set others to NaN
    w_stance(foot_contact(:,leg_id) < 0.5) = NaN;
    % Swing: keep data where contact < 0.5, set others to NaN
    w_swing(foot_contact(:,leg_id) >= 0.5) = NaN;
    % Plot stance in green, swing in blue
    plot(time_vector, w_stance, colors_stance, 'DisplayName','Stance');
    plot(time_vector, w_swing,  colors_swing,  'DisplayName','Swing');
    xlabel('Time (s)');
    ylabel(['\omega_', axis_names{i}, ' (rad/s)']);
    title(['IMU angular velocity Leg ', ' (', leg_names{leg_id}, ...
           ') - Axis: ', axis_names{i}]);
    legend('show');
end

%% ========== 1) PLOT JOINT ANGLES ==========
% Choose which leg to plot (1=FL, 2=FR, 3=RL, 4=RR)
leg_id    = 1;  
% Joint names 
joint_names = {'hip','thigh','calf'};
% foot contact Data
contact_leg = resampled_data.foot_contact.Data(:, leg_id); % Nx1: stance vs swing for chosen leg
col_base = (leg_id-1)*3;  % Base index for columns for this leg

figure('Name',['Joint Angles - Leg ', num2str(leg_id), ' (', leg_names{leg_id}, ')']);
for j = 1:3   
    subplot(3,1,j); 
    hold on; grid on;
    % Create stance/swing versions:
    angle_stance = resampled_data.j_ang.Data(:, col_base + j);      
    angle_swing  = resampled_data.j_ang.Data(:, col_base + j);       
    % Set swing samples to NaN in angle_stance
    angle_stance(contact_leg < 0.5) = NaN;
    % Set stance samples to NaN in angle_swing
    angle_swing(contact_leg >= 0.5) = NaN;
    % Plot
    plot(time_vector, angle_stance, colors_stance, 'DisplayName',[joint_names{j}, ' (Stance)']);
    plot(time_vector, angle_swing,  colors_swing,  'DisplayName',[joint_names{j}, ' (Swing)']);
    xlabel('Time (s)');
    ylabel(['\phi_{', joint_names{j}, '} (rad)']);
    legend('show');
    title(['Joint Angle - ', joint_names{j}, ' (', leg_names{leg_id}, ')']);
end


%% ========== 2) PLOT JOINT ANGULAR VELOCITIES ==========
figure('Name',['Joint Angular Velocities - Leg ', num2str(leg_id), ' (', leg_names{leg_id}, ')']);
for j = 1:3
    subplot(3,1,j);
    hold on; grid on;
    % Create stance/swing versions:
    vel_stance = resampled_data.j_vel.Data(:, col_base + j);
    vel_swing  = resampled_data.j_vel.Data(:, col_base + j);
    % Mask data
    vel_stance(contact_leg < 0.5) = NaN;
    vel_swing(contact_leg >= 0.5) = NaN;
    % Plot
    plot(time_vector, vel_stance, colors_stance, 'DisplayName',[joint_names{j}, ' (Stance)']);
    plot(time_vector, vel_swing,  colors_swing,  'DisplayName',[joint_names{j}, ' (Swing)']);
    xlabel('Time (s)');
    ylabel(['d\phi_{', joint_names{j}, '} (rad/s)']);
    legend('show');
    title(['Joint Angular Velocity - ', joint_names{j}, ' (', leg_names{leg_id}, ')']);
end
