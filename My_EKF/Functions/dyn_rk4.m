function xn1 = dyn_rk4(xn, un, dt, dynfunc)

if ~iscolumn(xn)
    xn = xn';
end
if ~iscolumn(un)
    un = un';
end


k1 = dynfunc(xn,un);
k2 = dynfunc(xn+dt*k1/2,un);
k3 = dynfunc(xn+dt*k2/2,un);
k4 = dynfunc(xn+dt*k3,un);

xn1 = xn + 1/6*dt*(k1 + 2*k2 + 2*k3 + k4);


end