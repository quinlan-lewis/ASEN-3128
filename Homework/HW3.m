clear all; clc; close all

m = .068; %[kg]
g = 9.8; %[m/s^2]
R = .06; %[m] radial distance from CG to prop
k_m = 0.0024; %[N*m/(N)] control moment coeffcient

%creating MOI matrix
Ix = 6.8 * 10^(-5); %[kg*m^2] body x-axis moment of inertia
Iy = 9.2 * 10^(-5); %[kg*m^2] body y-axis MOI
Iz = 1.35 * 10^(-4); %[kg*m^2] body z-axis MOI
I_b = [Ix 0 0; 0 Iy 0; 0 0 Iz]; %MOI matrix

v_aero = 1 * 10^(-3); %[N/(m/s)^2] aerodynamic force coefficient
mew = 2 * 10^(-6); %[N*m/(rad/s)^2] aerodynamic moment coefficient

x = 100;
y = 100;
z = -1600;
phi = .17;
theta = .17;
psi = 2.4;
u = 1;
v = -1;
w = 0;
p = 0;
q = 0;
r = 0;

xdot = .021;
ydot = 1.37;
zdot = -.34;
phidot = 0;
thetadot = 0;
psidot = 0;
udot = 1.72;
vdot = 1.7;
wdot = -.3;
pdot = -20.79;
qdot = -15.37;
rdot = 0;

v_b = [u;v;w];
X = -v_aero*(norm(v_b))*v_b(1);
Y = -v_aero*(norm(v_b))*v_b(2);
Z = -v_aero*(norm(v_b))*v_b(3);

L = -mew*norm([p;q;r])*p;
M = -mew*norm([p;q;r])*q;
N = -mew*norm([p;q;r])*r;


Zc = (wdot - q*u + p*v - g*cos(theta)*cos(phi) - Z/m)*m;
Lc = (pdot - ((Iy-Iz)*q*r/Ix))*Ix
Mc = (qdot - ((Iz-Ix)*p*r/Iy))*Iy
Nc = (rdot - ((Ix-Iy)*p*q/Iz))*Iz

Forces = [-1 -1 -1 -1; -R/sqrt(2) -R/sqrt(2) R/sqrt(2) R/sqrt(2); R/sqrt(2) -R/sqrt(2) -R/sqrt(2) R/sqrt(2); k_m -k_m k_m -k_m]^-1*[Zc;Lc;Mc;Nc]
