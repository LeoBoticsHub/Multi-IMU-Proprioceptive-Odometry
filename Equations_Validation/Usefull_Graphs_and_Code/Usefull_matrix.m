clear all
close all
clc
%% Rotation Matrix between world and robot body
syms y p r 
Ry = [cos(y) -sin(y) 0;
      sin(y) cos(y)  0;
      0    0  1];
Rp = [cos(p)  0  sin(p);
      0   1   0;
      -sin(p)  0 cos(p)];
  
Rr = [1  0  0 ;
      0  cos(r)  -sin(r);
      0  sin(r)   cos(r)];
R = Ry*Rp*Rr;

%% Skew 
syms x1 x2 x3
X=[0 -x3 x2;
    x3 0 -x1;
    -x2 x1 0];

%% Trasformation matrix between robot body and feet 
syms lc lt ox oy d t1 t2 t3 
R_bh=[1 0 0 ox;
    0 cos(t1) -sin(t1) oy;
    0 sin(t1) cos(t1) 0;
    0 0 0 1];
R_ht=[cos(t2) 0 sin(t2) 0;
    0 1 0 d;
    -sin(t2) 0 cos(t2) 0;
    0 0 0 1];
R_tc=[cos(t3) 0 sin(t3) 0;
    0 1 0 0;
    -sin(t3) 0 cos(t3) -lt;
    0 0 0 1];
R_cf=[1 0 0 0;
    0 1 0 0;
    0 0 1 -lc;
    0 0 0 1];
    
R_tot=simplify(R_bh*R_ht*R_tc*R_cf);

% Forward Kinematic
p_bf=R_tot(1:3,4)
% Rotation Matrix
R_bf=R_tot(1:3,1:3)

%% Velocity Jacobian
J_vel=jacobian(p_bf(:),[t1,t2,t3])

%% Angular Velocity Jacobian
J_om=[1 0 0;
    0 cos(t1) cos(t1);
    0 sin(t1) sin(t1)]
%% Position Hessian
J_dot_vel=simplify(jacobian(J_vel(:),[t1,t2,t3]))






