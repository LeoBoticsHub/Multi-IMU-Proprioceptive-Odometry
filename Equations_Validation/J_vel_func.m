function jacobian = J_vel_func(in1,lc,in3)
% derivative of the forward kinematic with respect to the three joint angles t1,t2,t3
d = in3(3,:);
lt = in3(4,:);
t1 = in1(1,:);
t2 = in1(2,:);
t3 = in1(3,:);
t5 = cos(t1);
t6 = cos(t2);
t7 = cos(t3);
t8 = sin(t1);
t9 = sin(t2);
t10 = sin(t3);
t11 = t2+t3;
t12 = cos(t11);
t13 = lt.*t9;
t14 = sin(t11);
t15 = lc.*t12;
t16 = lc.*t14;
t17 = -t15;
t18 = t13+t16;
jacobian = reshape([0.0,-d.*t8+lt.*t5.*t6+lc.*t5.*t6.*t7-lc.*t5.*t9.*t10,d.*t5+lt.*t6.*t8+lc.*t6.*t7.*t8-lc.*t8.*t9.*t10,t17-lt.*t6,-t8.*t18,t5.*t18,t17,-t8.*t16,t5.*t16],[3,3]);
