function X =skew(x)
X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end
% generates the antisymmetric matrix starting from a vector x=[x1;x2;x3] for
% being able to perform the vector product between two vectors e.g. u x v