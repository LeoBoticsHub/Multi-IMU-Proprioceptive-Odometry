function xn1 = dyn_rk4(xn, dt, dynfunc)

if ~iscolumn(xn)
    xn = xn';
end


k1 = dynfunc(xn);
k2 = dynfunc(xn+dt*k1/2);
k3 = dynfunc(xn+dt*k2/2);
k4 = dynfunc(xn+dt*k3);

xn1 = xn + 1/6*dt*(k1 + 2*k2 + 2*k3 + k4);


end