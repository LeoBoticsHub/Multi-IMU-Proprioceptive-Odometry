function xdot = process(x)

% The State of the system is as follows:
% State X
%   - position          (1:3)   body position
%   - velocity          (4:6)   body velocity
%   - acceleration      (7:9)   body acceleration
%   - euler angle       (10:12) body orientation Tait-Bryan
%   - angular velocity  (13:15) body angular velocity
%   - foot1 pos         (16:18) FL foot position
%   - foot1 vel         (19:21) FL foot velocity
%   - foot1 acc         (22:24) FL foot acceleration
%   - foot2 pos         (25:27) FR foot position
%   - foot2 vel         (28:30) FR foot velocity
%   - foot2 acc         (31:33) FR foot acceleration
%   - foot3 pos         (34:36) RL foot position
%   - foot3 vel         (37:39) RL foot velocity
%   - foot3 acc         (40:42) RL foot acceleration
%   - foot4 pos         (43:45) RR foot position
%   - foot4 vel         (46:48) RR foot velocity
%   - foot4 acc         (49:51) RR foot acceleration
%   - body acc bias     (52:54)
%   - body gyro bias    (55:57)
%   - foot1 acc bias    (58:60)
%   - foot1 gyro bias   (61:63)
%   - foot2 acc bias    (64:66)
%   - foot2 gyro bias   (67:69)
%   - foot3 acc bias    (70:72)
%   - foot3 gyro bias   (73:75)
%   - foot4 acc bias    (76:78)
%   - foot4 gyro bias   (79:81)
% all biases are in body frame

if ~iscolumn(x)
    x = x';
end

vel = x(4:6);
acc_body = x(7:9);
euler = x(10:12);
ang_vel = x(13:15);
foot1_vel = x(19:21);
foot1_acc = x(22:24);
foot2_vel = x(28:30);
foot2_acc = x(31:33);
foot3_vel = x(37:39);
foot3_acc = x(40:42);
foot4_vel = x(46:48);
foot4_acc = x(49:51);

deuler = mtx_w_to_euler_dot(euler)*ang_vel;

% Dynamics equations (update these with your specific dynamics)
xdot = [vel;acc_body;zeros(3,1);   % Add your dynamics equations for d_euler, foot velocities, and accelerations
        deuler;zeros(3,1);
        foot1_vel;foot1_acc;zeros(3,1);
        foot2_vel;foot2_acc;zeros(3,1);
        foot3_vel;foot3_acc;zeros(3,1);
        foot4_vel;foot4_acc;zeros(3,1);
        zeros(3,1);
        zeros(3,1);          
        zeros(3,1);
        zeros(3,1);
        zeros(3,1);
        zeros(3,1);
        zeros(3,1);
        zeros(3,1);
        zeros(3,1);
        zeros(3,1)];
end
