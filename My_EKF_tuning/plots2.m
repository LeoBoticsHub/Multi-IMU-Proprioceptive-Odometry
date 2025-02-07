% Create a figure occupying the full screen 
figureName = ['proc_n_ba = ', num2str(t),' proc_n_bg = ', num2str(i)];
figure('Name',figureName,'NumberTitle','off',...
       'Units','normalized','OuterPosition',[0 0 1 1]);
% Create a 3x2 layout (3 rows, 2 columns) for the main plots
T = tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

% ------------------ TILE #1 (TOP-LEFT): 3D or XY Position ------------------
ax1 = nexttile(T,1); 
hold(ax1,'on');
grid(ax1,'on');
plot3(ax1,resampled_data.pos_mocap.Data(param.data_start_idx:param.data_end_idx,1),resampled_data.pos_mocap.Data(param.data_start_idx:param.data_end_idx,2), resampled_data.pos_mocap.Data(param.data_start_idx:param.data_end_idx,3), 'LineWidth',1.3);
plot3(ax1, state(1,:), state(2,:), state(3,:), 'LineWidth',1.3);
title(ax1,'Robot Position (X-Y) over Time');
legend(ax1,"Ground Truth","EKF","Location","northeast");
axes(ax1);           
axis equal;
view(-0,90);
xlabel(ax1,"X (m)");
ylabel(ax1,"Y (m)");

% ------------------ TILE #2 (TOP-RIGHT): Z vs. Time ------------------
ax2 = nexttile(T,2);
hold(ax2,'on');
grid(ax2,'on');
plot(ax2, resampled_data.om_b_IMU.Time(param.data_start_idx:param.data_end_idx), resampled_data.pos_mocap.Data(param.data_start_idx:param.data_end_idx,3), 'LineWidth',1.3);
plot(ax2, resampled_data.om_b_IMU.Time(param.data_start_idx:param.data_end_idx), state(3,1:end-1)', 'LineWidth',1.3);
legend(ax2,"Ground Truth","EKF","Location","southeast");
title(ax2,'Robot Height (Z) over Time');
xlabel(ax2,"Time (s)");
ylabel(ax2,"Z (m)");

% ------------------ TILE #3 (BOTTOM-LEFT): Euler Angles in a Nested Layout -------------
T2 = tiledlayout(T,3,1,'TileSpacing','compact','Padding','compact');
T2.Layout.Tile = 3;  % Assign this nested layout to the 3rd tile in the main layout
title(T2,'Body Orientation (Euler Angles)','FontWeight','bold');
angleNames = {'\theta_r','\theta_p','\theta_y'};
axisNames = {'x','y','z'};
for i = 1:3
    axOrient = nexttile(T2,i);
    hold(axOrient,'on'); 
    grid(axOrient,'on');
    plot(axOrient, resampled_data.om_b_IMU.Time(param.data_start_idx:param.data_end_idx), resampled_data.orient_mocap_euler.Data(param.data_start_idx:param.data_end_idx,i), 'LineWidth',1);
    plot(axOrient, resampled_data.om_b_IMU.Time(param.data_start_idx:param.data_end_idx), state(9+i,1:end-1)', 'LineWidth',1.3);
    xlabel(axOrient,'Time (s)');
    ylabel(axOrient,[angleNames{i}, ' (rad)']);
    title(axOrient,['Orientation Axis ', axisNames{i}]);
end

% ------------------ TILE #4 (BOTTOM-RIGHT): Angular Velocity in a Nested Layout -----------
T3 = tiledlayout(T,3,1,'TileSpacing','compact','Padding','compact');
T3.Layout.Tile = 4;  % Assign this nested layout to the 4th tile in the main layout
title(T3,'Body Angular Velocity','FontWeight','bold');
axisNames = {'x','y','z'};
for i = 1:3
    axVel = nexttile(T3,i);
    hold(axVel,'on'); 
    grid(axVel,'on');
    plot(axVel, resampled_data.om_b_IMU.Time(param.data_start_idx:param.data_end_idx), resampled_data.om_b_IMU.Data(param.data_start_idx:param.data_end_idx,i), 'LineWidth',1);
    plot(axVel, resampled_data.om_b_IMU.Time(param.data_start_idx:param.data_end_idx), state(12+i,1:end-1)', 'LineWidth',1.3);
    xlabel(axVel,'Time (s)');
    ylabel(axVel,['\omega_{', axisNames{i}, '} (rad/s)']);
    title(axVel,['Angular Vel - Axis ', axisNames{i}]);
end

% ------------------ TILE #5 and #6 (BOTTOM ROW): Body Linear Velocity -------------
T4 = tiledlayout(T,3,1,'TileSpacing','compact','Padding','compact');
T4.Layout.Tile = [5 6];  % Span across the entire bottom row
title(T4,'Body Linear Velocity from EKF vs Ground Truth')
velocity={'V_x','V_y','V_z'};
for i = 1:3
    axVel = nexttile(T4,i);
    hold(axVel,'on');
    grid(axVel,'on');
    plot(axVel, resampled_data.om_b_IMU.Time(param.data_start_idx:param.data_end_idx), mocap_vel(param.data_start_idx:param.data_end_idx,i), 'LineWidth',1);
    plot(axVel, resampled_data.om_b_IMU.Time(param.data_start_idx:param.data_end_idx), state(3+i, 1:end-1)', 'LineWidth',1.3);
    xlabel(axVel,'Time (s)');
    ylabel(axVel,[velocity{i}, ' (m/s)']);
    title(axVel,['Velocity Comparison - Axis ', num2str(i)]);
end
