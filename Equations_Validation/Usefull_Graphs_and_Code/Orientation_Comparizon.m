%% Grafico dell'orientamento
steps = size(re_sensor_data.accel_body_IMU.Time,1);
init_idx = param.data_start_idx;
end_idx =  steps-1;
N = end_idx - init_idx + 1; 
init_time = re_sensor_data.accel_body_IMU.Time(init_idx);
time_vector = re_sensor_data.gyro_body_IMU.Time(init_idx:end_idx);
figure;
for i = 1:3
    subplot(3, 1, i);
    plot(time_vector, mipo_state_list(i+6, 1:end-1)', 'b', 'DisplayName', 'Calculated');
    hold on;
    plot(time_vector, re_sensor_data.orient_mocap_euler.Data(200:end-1,i), 'r--', 'DisplayName', 'Ground Truth');
    xlabel('Tempo (s)');
    ylabel(['\angle_', num2str(i), ' (rad)']);
    legend;
    title(['Orientation comparizon - Asse ', num2str(i)]);
end
