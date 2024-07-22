% Eric W. Frew
% ASEN 3128
% ode45_Lab1.m
% Created: 8/28/20

function [returning] = ode45_Lab1(~, state)
x0 = state(1);
y0 = state(2);
z0 = state(3);

xdot = x0 + 2*y0 + z0;
ydot = x0 - 5*z0;
zdot = x0*y0 - y0^2 + 3*z0^3;

returning(1) = xdot;
returning(2) = ydot;
returning(3) = zdot;
returning = returning';
end