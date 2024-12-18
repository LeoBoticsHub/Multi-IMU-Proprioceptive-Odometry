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
resampled_data.joint_acc      = re_sensor_data.joint_acc;
%%

resampled_data.om_fl_IMU.Data=sgolayfilt(resampled_data.om_fl_IMU.Data, 2, 11);
resampled_data.om_fr_IMU.Data=sgolayfilt(resampled_data.om_fr_IMU.Data, 2, 11);
resampled_data.om_rl_IMU.Data=sgolayfilt(resampled_data.om_rl_IMU.Data, 2, 11);
resampled_data.om_rr_IMU.Data=sgolayfilt(resampled_data.om_rr_IMU.Data, 2, 11);
resampled_data.om_b_IMU.Data=sgolayfilt(resampled_data.om_b_IMU.Data, 2, 11);