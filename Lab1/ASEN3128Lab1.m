% Eric W. Frew
% ASEN 3128
% ASEN3128Lab1.m
% Created: 8/28/20

%Lab 1 Simulate Equations of Motion EOM
%ASEN 3128 Lab 1 Simulate Equations of Motion EOM
%Cameron Mitchell, Tyler Gaston, Quinn Lewis, Eric Tate
clear all; close all; clc;

%% Problem 1
% Simulate the following set of equations with non-zero initial conditions
% and duration
state(1)=1;
state(2)=2;
state(3)=3;
state = state';

tspan=[0 20];
        
% xPrimeFun = x+y^2+z;
% yPrimeFun = x-5*z;
% zPrimeFun = x*y-y^2+3*z^2;

[T, data] = ode45(@ode45_Lab1, tspan, state);

figure('Name', 'Function Plots')
subplot(3,1,1) 
plot(T, data(:,1))
title('X vs. time')
subplot(3,1,2)
plot(T, data(:,2))
title('Y vs. time')
subplot(3,1,3)
semilogy(T, data(:,3))
title('Z vs. time')

%% Problem 2
% Construct simulation of the translational dynamics of a ball through air,
% where the forces on the body are not a function of the body attitude, but
% include drag(acts opposite inertial velocity vecotr) and gravity.

mass=30/1000;            %[g]
diameter = .03;     %[m]
Cd=0.6;
rho = 1.14;
A = pi*(diameter/2)^2;
wind = [0;0;0];
t = [0 5];
gravity = 9.8;

x_0 = 0;
y_0 = 0;
z_0 = 0;
u_0 = 0;
v_0 = 20;
w_0 = -20;
state = [x_0 y_0 z_0; u_0 v_0 w_0]';

