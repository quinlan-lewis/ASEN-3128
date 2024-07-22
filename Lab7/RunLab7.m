
clc
clear all;
close all;

recuv_tempest;

Va_trim = 21;
h_trim = 1800;

wind_inertial = [0;0;0];

trim_definition = [Va_trim; h_trim];

%%% Use full minimization to determine trim
[trim_variables, fval] = CalculateTrimVariables(trim_definition, aircraft_parameters);
[trim_state, trim_input]= TrimStateAndInput(trim_variables, trim_definition);
[Alon, Blon, Alat, Blat] = AircraftLinearModel(trim_definition, trim_variables, aircraft_parameters);

%% Question 2
%Problem a/b
Ayaw = Alat;
Ayaw(:,3) = Ayaw(:,3) - Blat(:,2);
[numR, denR] = ss2tf(Ayaw, Blat(:,2), [0 0 1 0 0 0], 0);
sys = tf(numR,denR);
sys2 = tf(-numR,denR);

figure(1);
hold on
rlocus(sys);
rlocus(sys2);
title('2a: Root Locus for Yaw Damper');

%Problem c/d
aircraft_state0 = trim_state;
control_input0 = trim_input;

[Vlat Dlat] = eig(Alat);
aircraft_state0(8) = aircraft_state0(8) + real(Vlat(1,4));
aircraft_state0(10) = aircraft_state0(10) + real(Vlat(2,4));
aircraft_state0(12) = aircraft_state0(12) + real(Vlat(3,4));
aircraft_state0(4) = aircraft_state0(4) + real(Vlat(4,4));
aircraft_state0(6) = aircraft_state0(6) + real(Vlat(5,4));

%With Yaw damper 
tfinal = 10;
TSPAN = [0 tfinal];
[TOUT2,YOUT2] = ode45(@(t,y) AircraftEOMControl(t,y,control_input0,wind_inertial,aircraft_parameters,2),TSPAN,aircraft_state0,[]);

for i=1:length(TOUT2)
    UOUT2(i,:) = control_input0';
end

PlotAircraftSim(TOUT2,YOUT2,UOUT2,wind_inertial,'b')


%Without Yaw Damper
tfinal = 10;
TSPAN = [0 tfinal];
[TOUT3,YOUT3] = ode45(@(t,y) AircraftEOM(t,y,control_input0,wind_inertial,aircraft_parameters),TSPAN,aircraft_state0,[]);

for i=1:length(TOUT3)
    UOUT3(i,:) = control_input0';
end

PlotAircraftSim(TOUT3,YOUT3,UOUT3,wind_inertial,'r')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper code for Problem 3. Look inside the AircraftEOMControl function
% for hints on how to set up controllers for Problem 2 and Problem 4.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Transfer function from elevator to pitch angle
[num_elev2pitch, den_elev2pitch] = ss2tf(Alon(1:4,1:4), Blon(1:4,1), [0 0 0 1],0);

%%% Controller
kq = 1; %NOT REASONABLE VALUES
kth = 1; %NOT REASONABLE VALUES
num_c = [kq kth];
den_c = 1;

%%% Closed loop transfer function
pitch_cl = feedback(1, tf(conv(num_c, num_elev2pitch), conv(den_c, den_elev2pitch)));
[num_cl, den_cl] = tfdata(pitch_cl,'v');

%%% Poles of the closed loop (linear) system. Now do the same with the
%%% state stpace model.
roots(den_cl);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%% Full sim in ode45
aircraft_state0 = trim_state;
control_input0 = trim_input;

tfinal = 200;
TSPAN = [0 tfinal];
[TOUT2,YOUT2] = ode45(@(t,y) AircraftEOMControl(t,y,control_input0,wind_inertial,aircraft_parameters),TSPAN,aircraft_state0,[]);


for i=1:length(TOUT2)
    UOUT2(i,:) = control_input0';
end

PlotAircraftSim(TOUT2,YOUT2,UOUT2,wind_inertial,'b')

%% Question 3
[num_elev2pitch, den_elev2pitch] = ss2tf(Alon(1:4,1:4), Blon(1:4,1), [0 0 0 1],0);
kq = 1; %NOT REASONABLE VALUES
kth = 1; %NOT REASONABLE VALUES
num_c = [kq kth];
den_c = 1;
pitch_cl = feedback(1, tf(conv(num_c, num_elev2pitch), conv(den_c, den_elev2pitch)));
[num_cl, den_cl] = tfdata(pitch_cl,'v');
roots(den_cl);
rlocus(pitch_cl);

Lecture Version
[numG, denG] = tfdata(ss2tf(Alon(1:4,1:4), Blon(1:4, 1), [0 0 0 1], 0));
numCG= conv(numC, numG);
denCG= conv(denC, denG);
sysCL= feedback(tf(numCG, denCG), 1);
rlocus(sysCL);

%Test Pitch Controller
aircraft_state0 = trim_state;
control_input0 = trim_input;

[Vlon, Dlon] = eig(Alon(1:4,1:4));

aircraft_state0(7) = aircraft_state0(7) + real(Vlon(1,4));
aircraft_state0(9) = aircraft_state0(9) + real(Vlon(2,4));
aircraft_state0(11) = aircraft_state0(11) + real(Vlon(3,4));
aircraft_state0(5) = aircraft_state0(5) + real(Vlon(4,4));

%With Pitch Controller
tfinal = 100;
TSPAN = [0 tfinal];
[TOUT4,YOUT4] = ode45(@(t,y) AircraftEOMControl(t,y,control_input0,wind_inertial,aircraft_parameters,1),TSPAN,aircraft_state0,[]);

for i=1:length(TOUT4)
    UOUT4(i,:) = control_input0';
end

PlotAircraftSim(TOUT4,YOUT4,UOUT4,wind_inertial,'b')

 %Without Pitch Damper
tfinal = 100;
TSPAN = [0 tfinal];
[TOUT5,YOUT5] = ode45(@(t,y) AircraftEOM(t,y,control_input0,wind_inertial,aircraft_parameters),TSPAN,aircraft_state0,[]);

for i=1:length(TOUT5)
    UOUT5(i,:) = control_input0';
end

PlotAircraftSim(TOUT5,YOUT5,UOUT5,wind_inertial,'r')

