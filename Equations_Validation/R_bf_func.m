function R_bf = R_bf_func(in1)
t1 = in1(1,:);
t2 = in1(2,:);
t3 = in1(3,:);
t5 = cos(t1);
t6 = sin(t1);
t7 = t2+t3;
t8 = cos(t7);
t9 = sin(t7);
R_bf = reshape([t8,t6.*t9,-t5.*t9,0.0,t5,t6,t9,-t6.*t8,t5.*t8],[3,3]);
%R_bf =[ cos(t2 + t3), sin(t2 + t3)*sin(t1), sin(t2 + t3)*cos(t1);
%             0,              cos(t1),             -sin(t1);
% -sin(t2 + t3), cos(t2 + t3)*sin(t1), cos(t2 + t3)*cos(t1)];
% % Define symbolic variables for joint angles
% syms t1 t2 t3
% 
% % Rotation matrix around X-axis
% Rx = [1, 0, 0;
%       0, cos(t1), -sin(t1);
%       0, sin(t1), cos(t1)];
% 
% % Rotation matrix around Y-axis
% Ry_theta2 = [cos(t2), 0, sin(t2);
%              0, 1, 0;
%              -sin(t2), 0, cos(t2)];
% 
% Ry_theta3 = [cos(t3), 0, sin(t3);
%              0, 1, 0;
%              -sin(t3), 0, cos(t3)];
% 
% % Combined rotation matrix
% R = Rx * Ry_theta2 * Ry_theta3;
% 
% % Simplify the result
% R_bf = simplify(R);