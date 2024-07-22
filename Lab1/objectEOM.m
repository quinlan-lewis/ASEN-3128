function xDot = objectEOM(t,x,rho,Cd,A,mass,gravity,wind)
%Inputs:
%t= time [s]
%x= position vector [x1;x2;x3]
%rho= density [kg/m^3]
%Cd= coefficient of drag
%A = cross sectional area of golf ball [m^2]
%mass = mass of golf ball [g]
%wind = wind velocity vector [w1;w2;w3]

p = x(:,1);
v = x(:,2);
Drag = (1/2).*Cd.*rho.*(v.^2).*A; %[N]

pDot = v;



end