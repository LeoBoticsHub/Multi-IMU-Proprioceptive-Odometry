function xdot = process(x,u)

% The State of the system is as follows:
% State X
%   - position          (1:3)   body position
%   - velocity          (4:6)   body velocity
%   - acceleration      (7:9)   body acceleration
%   - euler angle       (10:12) body orientation Tait-Bryan
%   - angular velocity  (13:15) body angular velocity
%   - foot1 pos         (16:18) FL foot position
%   - foot1 vel         (19:21) FL foot velocity
%   - foot2 pos         (22:24) FR foot position
%   - foot2 vel         (25:27) FR foot velocity
%   - foot3 pos         (28:30) RL foot position
%   - foot3 vel         (31:33) RL foot velocity
%   - foot4 pos         (34:36) RR foot position
%   - foot4 vel         (37:39) RR foot velocity
%   - body acc bias     (40:42)
%   - body gyro bias    (43:45)
%   - foot1 acc bias    (46:48)
%   - foot1 gyro bias   (49:51)
%   - foot2 acc bias    (52:54)
%   - foot2 gyro bias   (55:57)
%   - foot3 acc bias    (58:60)
%   - foot3 gyro bias   (61:63)
%   - foot4 acc bias    (64:66)
%   - foot4 gyro bias   (67:69)
% all biases are in body frame

% Control U
%   - a1      (1:3)     foot 1 IMU acceleration (already in body frame)
%   - a2      (4:6)   foot 2 IMU acceleration (already in body frame)
%   - a3      (7:9)   foot 3 IMU acceleration (already in body frame)
%   - a4      (10:12)   foot 4 IMU acceleration (already in body frame)
%   - t       (13)      time 


% dot x 
%   - velocity    
%   - acc    
%   - d_euler 
%   - foot1 vel 
%   - foot1 acc 
%   - foot2 vel 
%   - foot2 acc 
%   - foot3 vel 
%   - foot3 acc 
%   - foot4 vel 
%   - foot4 acc 

if ~iscolumn(x)
    x = x';
end

if ~iscolumn(u)
    u = u';
end

pos = x(1:3);
vel = x(4:6);
acc_body = x(7:9);
euler = x(10:12);
ang_vel = x(13:15);
foot1_pos = x(16:18);
foot1_vel = x(19:21);
foot2_pos = x(22:24);
foot2_vel = x(25:27);
foot3_pos = x(28:30);
foot3_vel = x(31:33);
foot4_pos = x(34:36);
foot4_vel = x(37:39);
ba = x(40:42);     % body acc bias
bg = x(43:45);     % gyro bias
foot1_ba = x(46:48);     % foot 1 acc bias
foot1_bg = x(49:51);     % foot 1 gyro bias
foot2_ba = x(52:54);     % foot 2 acc bias
foot2_bg = x(55:57);     % foot 2 gyro bias
foot3_ba = x(58:60);     % foot 3 acc bias
foot3_bg = x(61:63);     % foot 3 gyro bias
foot4_ba = x(64:66);     % foot 4 acc bias
foot4_bg = x(67:69);     % foot 4 gyro bias

foot1_acc = u(1:3)   - foot1_ba;  % already changed to body frame
foot2_acc = u(4:6) - foot2_ba;  % already changed to body frame
foot3_acc = u(7:9) - foot3_ba;  % already changed to body frame
foot4_acc = u(10:12) - foot4_ba;  % already changed to body frame

deuler = mtx_w_to_euler_dot(euler)*ang_vel;
R_bw = euler_to_rot(euler); % rotation matrix from body to world frame 
acc = acc_body -[0;0;9.8];
foot1_acc_w = R_bw*foot1_acc - [0;0;9.8];
foot2_acc_w = R_bw*foot2_acc - [0;0;9.8];
foot3_acc_w = R_bw*foot3_acc - [0;0;9.8];
foot4_acc_w = R_bw*foot4_acc - [0;0;9.8];

xdot = [vel;acc;zeros(3,1);
        deuler;zeros(3,1);
        foot1_vel;foot1_acc_w;
        foot2_vel;foot2_acc_w;
        foot3_vel;foot3_acc_w;
        foot4_vel;foot4_acc_w;
        zeros(3,1);                  % body acc bias
        zeros(3,1);                  % gyro bias
        zeros(3,1);                  % foot 1 acc bias
        zeros(3,1);                  % foot 1 gyro bias
        zeros(3,1);                  % foot 2 acc bias
        zeros(3,1);                  % foot 2 gyro bias
        zeros(3,1);                  % foot 3 acc bias
        zeros(3,1);                  % foot 3 gyro bias
        zeros(3,1);                  % foot 4 acc bias
        zeros(3,1)];                 % foot 4 gyro bias
end