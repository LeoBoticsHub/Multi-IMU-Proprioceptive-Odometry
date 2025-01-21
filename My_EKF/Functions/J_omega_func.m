function J_omega = J_omega_func(in1)
t1 = in1(1,:);
t2 = cos(t1);
t3 = sin(t1);
J_omega = reshape([1.0,0.0,0.0,0.0,t2,t3,0.0,t2,t3],[3,3]);

