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

