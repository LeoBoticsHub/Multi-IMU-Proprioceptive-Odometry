function p_bf = p_fk_func(in1,lc,in3)

d = in3(3,:);
lt = in3(4,:);
ox = in3(1,:);
oy = in3(2,:);
t1 = in1(1,:);
t2 = in1(2,:);
t3 = in1(3,:);
t5 = cos(t1);
t6 = cos(t2);
t7 = cos(t3);
t8 = sin(t1);
t9 = sin(t2);
t10 = sin(t3);
p_bf = [ox-lt.*t9-lc.*sin(t2+t3);oy+d.*t5+lt.*t6.*t8+lc.*t6.*t7.*t8-lc.*t8.*t9.*t10;d.*t8-lt.*t5.*t6-lc.*t5.*t6.*t7+lc.*t5.*t9.*t10];
