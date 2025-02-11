function xn1 = dyn_rk4(xn, dt, dynfunc)

% Ensure the state vector xn is a column vector
if ~iscolumn(xn)
    xn = xn';
end

% Compute the four increments for RK4
k1 = dynfunc(xn);
k2 = dynfunc(xn+dt*k1/2);
k3 = dynfunc(xn+dt*k2/2);
k4 = dynfunc(xn+dt*k3);

% Update the state using RK4 formula
xn1 = xn + 1/6*dt*(k1 + 2*k2 + 2*k3 + k4);


end