function DCM = transformB2I(phi,theta,psi)
%     C_3 = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
%     C_2 = [cos(phi) 0 -sin(phi); 0 1 0; sin(phi) 0 cos(phi)];
%     C_1 = [1 0 0; 0 cos(theta) sin(theta); 0 -sin(theta) cos(theta)];
% 
%     DCM = (C_1*C_2*C_3)';
    
    DCM = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
        sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
        cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)]';
end