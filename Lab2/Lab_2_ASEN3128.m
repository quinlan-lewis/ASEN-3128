%% ASEN 3128: Lab 2 Simulate EOM
%Authors: Quinlan Lewis, Adriana Lysak, Gio Borrani, Bem Capeloto
%Last updated: 9/24/2020

%% Housekeeping
clear all; clc; close all
%% Purpose:
%Purpose of code is to simulate the state vector and how each of the 12
%components change with resepect to time as opposed to a 3D plot

%% Declaring givens/constants
m = .068; %[kg]
g = 9.8; %[m/s^2]
radius = .06; %[m] radial distance from CG to prop
k_m = 0.0024; %[N*m/(N)] control moment coeffcient

%creating MOI matrix
I_x = 6.8 * 10^(-5); %[kg*m^2] body x-axis moment of inertia
I_y = 9.2 * 10^(-5); %[kg*m^2] body y-axis MOI
I_z = 1.35 * 10^(-4); %[kg*m^2] body z-axis MOI
I_b = [I_x 0 0; 0 I_y 0; 0 0 I_z]; %MOI matrix

v_aero = 1 * 10^(-3); %[N/(m/s)^2] aerodynamic force coefficient
mew = 2 * 10^(-6); %[N*m/(rad/s)^2] aerodynamic moment coefficient

%% Problem 1 and 2a
%For problem 1 we created an EOM ode45 function that uses the equations we
%have been discussing in class and implements them into this lab to find
%the equation of motion for the quad copter. The plots for problem 1 are
%given in the document but for organization sake we added the control force
%and moment into the function here so we can see that there is no change
%between the problem using control forces and the problem not using control
%forces.
%assigning inital values for the state vector
x = 0;
y = 0;
z = 0;
phi = 0;
theta = 0;
psi = 0;
u = 0;
v = 0;
w = 0;
p = 0;
q = 0;
r = 0;

state = [x,y,z,phi,theta,psi,u,v,w,p,q,r]';

%control values for inputs, control force is gravity to make sure the quad
%rotor stays in the air and keeps its xyz position constant
Zc = m*g;
f1 = Zc/4;
f2 = f1;
f3 = f1;
f4 = f1;
control = [f1;f2;f3;f4]; %input force split amongst the 4 rotors
tspan = [0 10];

[Tout, State_out] = ode45(@(t, S) quadEOM(tspan,state,m,radius,k_m,I_x,I_y,I_z,v_aero,mew,control), tspan, state);
EOMplotter(Tout, State_out)

%% Problem 2b
%for 2b we needed to find both a control force that would cause the copter
%to translate 5 m/s to the east and needed to find that roll angle. To do
%so we solved the equations of motion on a seperate piece of paper to get
%equations for phi and Zc that we can use to solve for those 2 unknowns.
%Here roll is positive cause it is facing forward still and is titlting to
%the east so it can move in that direction.
x = 0;
y = 0;
z = 0;
phi = atan(v_aero*25/(m*g));
theta = 0;
psi = 0;
u = 0;
v = 5*cos(phi);
w = -5*sin(phi);
p = 0;
q = 0;
r = 0;

state = [x,y,z,phi,theta,psi,u,v,w,p,q,r]';

%control values for inputs
Zc = -m*(-v_aero*25*sin(phi)/m - g*cos(phi));
f1 = Zc/4;
f2 = f1;
f3 = f1;
f4 = f1;
control = [f1;f2;f3;f4]; %input force, split amongst the 4 motors
tspan = [0 10];

[Tout, State_out2] = ode45(@(t, S) quadEOM(tspan,state,m,radius,k_m,I_x,I_y,I_z,v_aero,mew,control), tspan, state);
EOMplotter(Tout,State_out2)

%% Problem 2c
%For this problem we needed to again find values for the control moment but
%instead of the roll angle we needed to find a value for the pitch angle.
%These values have the same magnitude as problem 2b but are now oriented
%differently because of the 90 deg yaw angle. This causes the copter to now
%be faced in the east direction so its u value should now be the motion it
%is moving in because its orientation has changed due to the yaw angle. The
%pitch angle is also negative because it needs to pitch down in order to
%move forward in which this case East is now forward.
x = 0;
y = 0;
z = 0;
phi = 0;
theta = -atan(v_aero*25/(m*g));
psi = pi/2;
u = 5*cos(theta);
v = 0;
w = 5*sin(theta);
p = 0;
q = 0;
r = 0;

state = [x,y,z,phi,theta,psi,u,v,w,p,q,r]';

%control values for inputs
Zc = -m*(-v_aero*25*sin(abs(theta))/m - g*cos(abs(theta)));
f1 = Zc/4;
f2 = f1;
f3 = f1;
f4 = f1;
control = [f1;f2;f3;f4]; %input force, split amongst the 4 rotors
tspan = [0 10];

[Tout, State_out3] = ode45(@(t, S) quadEOM(tspan,state,m,radius,k_m,I_x,I_y,I_z,v_aero,mew,control), tspan, state);
EOMplotter(Tout,State_out3)

%% Problem 3
%For this problem we were tasked we proving the stability, or in this case
%the instability of the copter. We did this picking random values for uvw
%and phi theta and psi, to see how they would effect the system. For some
%reason our code does not show a change in the xyz direction which we think
%is wrong, but it does show an increase in the uvw velocities which is
%expected as seen from teh example video.
%assigning inital values for the state vector
x = 0;
y = 0;
z = 0;
phi = pi/3;
theta = 5*pi/6;
psi = 4*pi/3;
u = 2;
v = 3;
w = 5;
p = 0;
q = 0;
r = 0;

state = [x,y,z,phi,theta,psi,u,v,w,p,q,r]';

%control values for inputs
Zc = m*g;
f1 = Zc/4;
f2 = f1;
f3 = f1;
f4 = f1;
control = [f1;f2;f3;f4]; %input force, split amongst the 4 rotors
tspan = [0 10];

[Tout, State_out] = ode45(@(t, S) quadEOM(tspan,state,m,radius,k_m,I_x,I_y,I_z,v_aero,mew,control), tspan, state);
EOMplotter(Tout, State_out)

%% Functions
%EOM function:
function Result = quadEOM(~,state,m,radius,k_m,I_x,I_y,I_z,v_aero,mew,inputs)
    %Purpose of this function is to calculate the equations of motion using
    %the quadrotor equations given to us in class in lecture 5 slide 9.
    %These below equations are taken directly from the lecture slides.
    
    g = 9.8; %[m/s^2] gravity constant
    %control force and moments
    f1 = inputs(1); f2 = inputs(2); f3 = inputs(3); f4 = inputs(4);
    F_cntl = [0;0;(-f1 - f2 - f3 - f4)];
    M_cntl = [((radius/sqrt(2))*(-f1 - f2 + f3 + f4));((radius/sqrt(2))*(f1 - f2 - f3 + f4));(k_m*(f1-f2+f3-f4))];
    
    phi = state(4); theta = state(5); psi = state(6);
    
    %Below is calculating aerodynamic force from drag
    V_a = norm(state(7:9));
    V_b = [state(7); state(8); state(9)];
    F_aero = -v_aero*V_a*V_b; %aerodynamic force
    X = F_aero(1);
    Y = F_aero(2);
    Z = F_aero(3);
    
    %Below is calculating the aerodynamic moments acting on the quadrotor
    ang_vel = [state(10); state(11); state(12)];
    M_b = -mew*(norm(state(10:12)))*ang_vel;
    L = M_b(1);
    M = M_b(2);
    N = M_b(3);
    
    %assigning equations for x y and z dot
    xyz_dot = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
                cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
                -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)]*[state(7);state(8);state(9)];
    xdot = xyz_dot(1);
    ydot = xyz_dot(2);
    zdot = xyz_dot(3);
    
    %assigning angle rates
    angle_dot = [1, sin(phi)*tan(theta), cos(phi)*tan(theta); 0, cos(phi), -sin(phi); 0, sin(phi)*(1/cos(theta)), cos(phi)*(1/cos(theta))]...
        *[state(10);state(11);state(12)];
    phi_dot = angle_dot(1);
    theta_dot = angle_dot(2);
    psi_dot = angle_dot(3);
    
    %assigning uvw_dot which are velocity rates
    p = state(10); q = state(11); r = state(12);
    u = state(7); v = state(8); w = state(9);
    uvw_dot = [r*v-q*w; p*w-r*u; q*u-p*v] + g*[-sin(theta); cos(theta)*sin(phi); cos(theta)*cos(phi)]...
        + (1/m)*[X;Y;Z] + (1/m)*F_cntl;
    u_dot = uvw_dot(1);
    v_dot = uvw_dot(2);
    w_dot = uvw_dot(3);
    
    %assigning pqr_dot which are angluar velocity rates
    pqr_dot = [((I_y-I_z)/I_x)*q*r; ((I_z-I_x)/I_y)*p*r; ((I_x-I_y)/I_z)*q*p] + ...
        [(1/I_x)*L; (1/I_y)*M; (1/I_z)*N] + ...
        [(1/I_x)*M_cntl(1); (1/I_y)*M_cntl(2); (1/I_z)*M_cntl(3)];
    p_dot = pqr_dot(1);
    q_dot = pqr_dot(2);
    r_dot = pqr_dot(3);
    
    %returning values for the state vector
    Result = [xdot; ydot; zdot; phi_dot; theta_dot; psi_dot; u_dot; v_dot; w_dot; p_dot; q_dot; r_dot];
end

%Plotting function:
function [] = EOMplotter(Tout,State_out2)
    %This function is only used as  a plotting tool for the different
    %components of the state vector and the equation of motions

    figure(1)
    subplot(3,1,1)
        plot(Tout, State_out2(:,1))
        xlabel('Time(s)')
        ylabel('x[m]')
        ylim([-1 1])
    subplot(3,1,2)
        plot(Tout, State_out2(:,2))
        xlabel('Time(s)')
        ylabel('y[m]')
        %ylim([0 60])
    subplot(3,1,3)
        plot(Tout, State_out2(:,3))
        xlabel('Time(s)')
        ylabel('z[m]')
        %ylim([-1 1])
    sgtitle('Position vs. Time')
    xlim([0 10])

    figure(2)
    subplot(3,1,1)
        plot(Tout, State_out2(:,4))
        xlabel('Time(s)')
        ylabel('Roll[rad]')
    subplot(3,1,2)
        plot(Tout, State_out2(:,5))
        xlabel('Time(s)')
        ylabel('Pitch[rad]')
    subplot(3,1,3)
        plot(Tout, State_out2(:,6))
        xlabel('Time(s)')
        ylabel('Yaw[rad]')
    sgtitle('Euler Angles vs. Time')
    %ylim([-1 1])

    figure(3)
    subplot(3,1,1)
        plot(Tout, State_out2(:,7))
        xlabel('Time(s)')
        ylabel('uE[m/s]')
        %ylim([-1 1])
    subplot(3,1,2)
        plot(Tout, State_out2(:,8))
        xlabel('Time(s)')
        ylabel('vE[m/s]')
        %ylim([3 8])
    subplot(3,1,3)
        plot(Tout, State_out2(:,9))
        xlabel('Time(s)')
        ylabel('wE[m/s]')
        %ylim([-1 1])
    sgtitle('Velocity vs. Time') 

    figure(4)
    subplot(3,1,1)
        plot(Tout, State_out2(:,10))
        xlabel('Time(s)')
        ylabel('p[rad/s]')
    subplot(3,1,2)
        plot(Tout, State_out2(:,11))
        xlabel('Time(s)')
        ylabel('q[rad/s]')
    subplot(3,1,3)
        plot(Tout, State_out2(:,12))
        xlabel('Time(s)')
        ylabel('r[rad/s]')
    sgtitle('Angular Velocity vs. Time')
    %ylim([-1 1])
    
    end

