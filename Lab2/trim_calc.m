function [force,p,vout] = trim_calc(v_a,theta,psi,m,v_aero)
    g = 9.81; %[m/s^2]
    
    syms phi Zc
    
    xyz_dot = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(theta)*sin(psi);...
            cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(theta);...
            -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)]*v_a;
    xdot = xyz_dot(1);
    ydot = xyz_dot(2);
    zdot = xyz_dot(3);
    
    f_aero = -v_aero*(norm(v_a))*v_a;
    
    u_dot = 0 == g*-sin(theta) + (1/m)*f_aero(1) + (1/m)*0;
    v_dot = 0 == g*cos(theta)*sin(phi) + (1/m)*f_aero(2) + (1/m)*0;
    w_dot = 0 == g*cos(theta)*cos(phi) + (1/m)*f_aero(3) + (1/m)*Zc;
    
    p = double(vpasolve(v_dot,phi));
    force = double(vpasolve(subs(w_dot,phi,p(1)),Zc));
    
    vout = [double(subs(xdot,phi,p)); double(subs(ydot,phi,p)); double(subs(zdot,phi,p))];
    
end

