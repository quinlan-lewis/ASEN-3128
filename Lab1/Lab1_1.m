%ASEN 3128 Lab 1 Simulate Equations of Motion EOM
%Problem 1
%Cameron Mitchell, Tyler Gaston, Quinn Lewis, Eric Tate
clear all; close all; clc;

%% Problem 1
% Simulate the following set of equations with non-zero initial conditions
% and duration
x0_1=[5;5;5]; %initial conditions for x, y, and z
tSpan1=[0 1]; %time span [s]

[Tout1, state1] = ode45(@eom, tSpan1, x0_1);  %ODE45 function call

%Below is creating the subplot of the x y and z graphs with respect to time
figure('Name', 'Function Plots')
subplot(3,1,1)
    plot(Tout1, state1(:,1))
    xlabel('Time(s)')
    ylabel('x-component')
subplot(3,1,2)
    plot(Tout1, state1(:,2))
    xlabel('Time(s)')
    ylabel('y-component')
subplot(3,1,3)
    semilogy(Tout1, state1(:,3))
    xlabel('Time(s)')
    ylabel('z-component')
sgtitle('Components vs. Time')      %title of group of subplots


%% functions
%1
function stateDot = eom(~, state)
%Inputs:
%t = time [s]
%state = Vector containing x,y,z values [x;y;z]
%Outputs:
%stateDot =[xPrime, yPrime, zPrime]
%xPrime = x+2*y+z;
%yPrime = x-5*z;
%zPrime = x*y-y^2+3*z^3;
%Methodology: Simulate the given set of equations (xPrime, yPrime, zPrime)
%with non-zero initial conditions and duration by using numerical
%integration through ode45. This function simply takes in a state vector
%that contains the initial conditions for the function and then using those
%initial conditions the ode45 function numerically integrates over the
%inputted time span

x=state(1);
y=state(2);
z=state(3);
xPrime = x+2*y+z;
yPrime = x-5*z;
zPrime = x*y-y^2+3*z^3;
stateDot = [xPrime; yPrime; zPrime];

end