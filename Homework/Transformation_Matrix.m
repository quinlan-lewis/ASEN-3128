%Quinlan Lewis
%ASEN 3128: Exam 1 Part 2
%9/16/20

clear all; clc;

%declare angles of rotation in rads
theta = .17; 
phi = 0;
psi = 1.3; 

R = transformI2B(phi,theta,psi);
vB = [15.8;0;2.8];
VI = R*vB;

%below is the function for transforming a matrix from inertial to body
%coordinates
function DCM = transformI2B(phi,theta,psi)
    DCM = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
        sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
        cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)-sin(phi)*cos(psi) cos(theta)*cos(theta)];
end
