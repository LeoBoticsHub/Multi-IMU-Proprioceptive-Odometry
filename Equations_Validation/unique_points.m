function [dt6,j_ang,j_vel]=unique_points(dt_list,j_ang,j_vel)
%deals with the non unique points in the dataset

% % Find non Unique points
% for i=1:k-1
%     if dt_list(i)<=0 
%     fprintf('dt_list is not strictly higher then zero at index %d \n',i);
%     end
% end

% j_ang
[unique_times_ang, ia_ang, ic_ang] = unique(j_ang.Time);
num_duplicates_ang = length(j_ang.Time) - length(unique_times_ang);
disp(['Number of duplicated Timestamps in j_ang: ', num2str(num_duplicates_ang)]);

% j_vel
[unique_times_vel, ia_vel, ic_vel] = unique(j_vel.Time);
num_duplicates_vel = length(j_vel.Time) - length(unique_times_vel);
disp(['Number of duplicated Timestamps in j_vel: ', num2str(num_duplicates_vel)]);

%% Missing Data
% for i=1:k
%     time(i,:) = double(messages{i}.header.stamp.sec) + double(messages{i}.header.stamp.nanosec)*10^-9;
%     for j=1:12
%         if isfield(messages{i}, 'linear_acceleration')
%             joint_vel(i,j) = messages{i}.velocity(j);
%         else
%             joint_vel(i,j) = NaN;
%         end
%     end
% end

%% Make the timeseries unique
% j_ang
[unique_times_ang, ia_ang, ~] = unique(j_ang.Time, 'stable'); % Find unique timestamps
unique_data_ang = j_ang.Data(ia_ang, :); 
j_ang = timeseries(unique_data_ang, unique_times_ang); % Ricreate the timeseries j_ang without duplications

% j_vel
[unique_times_vel, ia_vel, ~] = unique(j_vel.Time, 'stable'); % Find unique timestamps
unique_data_vel = j_vel.Data(ia_vel, :);
j_vel = timeseries(unique_data_vel, unique_times_vel); % Ricreate the timeseries j_vel without duplications

dt_list = diff(j_vel.Time);
dt6 = mean(dt_list);
