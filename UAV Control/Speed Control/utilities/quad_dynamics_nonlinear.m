function Quad = quad_dynamics_nonlinear(Quad)
    %% Update Accelerations

    Quad.X_ddot = (-[cos(Quad.phi)*sin(Quad.theta)*cos(Quad.psi)+sin(Quad.phi)*sin(Quad.psi)]*Quad.U1-Quad.Kdx*Quad.X_dot)/Quad.m;
    Quad.Y_ddot = (-[cos(Quad.phi)*sin(Quad.psi)*sin(Quad.theta)-cos(Quad.psi)*sin(Quad.phi)]*Quad.U1-Quad.Kdy*Quad.Y_dot)/Quad.m;
    Quad.Z_ddot = (-[cos(Quad.phi)*cos(Quad.theta)]*Quad.U1-Quad.Kdz*Quad.Z_dot)/Quad.m+Quad.g;

    Quad.p_dot = (Quad.q*Quad.r*(Quad.Jy - Quad.Jz) - Quad.Jp*Quad.p*Quad.Obar + Quad.l*Quad.U2)/Quad.Jx;
    Quad.q_dot = (Quad.p*Quad.r*(Quad.Jz - Quad.Jx) + Quad.Jp*Quad.q*Quad.Obar + Quad.l*Quad.U3)/Quad.Jy;
    Quad.r_dot = (Quad.p*Quad.q*(Quad.Jx - Quad.Jy) + Quad.U4)/Quad.Jz;

    Quad.phi_dot   = Quad.p + sin(Quad.phi)*tan(Quad.theta)*Quad.q + cos(Quad.phi)*tan(Quad.theta)*Quad.r;
    Quad.theta_dot = cos(Quad.phi)*Quad.q - sin(Quad.phi)*Quad.r;
    Quad.psi_dot   = sin(Quad.phi)/cos(Quad.theta)*Quad.q + cos(Quad.phi)/cos(Quad.theta)*Quad.r;

    %% Disturbance model

    Quad.X_ddot = Quad.X_ddot + Quad.X_dis/Quad.m; 
    Quad.Y_ddot = Quad.Y_ddot + Quad.Y_dis/Quad.m; 
    Quad.Z_ddot = Quad.Z_ddot + Quad.Z_dis/Quad.m; 
    Quad.phi_dot = Quad.phi_dot + Quad.phi_dis/Quad.Jx*Quad.Ts; 
    Quad.theta_dot = Quad.theta_dot + Quad.theta_dis/Quad.Jy*Quad.Ts; 
    Quad.psi_dot = Quad.psi_dot + Quad.psi_dis/Quad.Jz*Quad.Ts;

    %% Update Velocities and Positions

    % Calculating the Z velocity & position
    Quad.Z_dot = Quad.Z_ddot*Quad.Ts + Quad.Z_dot;
    Quad.Z = Quad.Z_dot*Quad.Ts + Quad.Z;

    % Calculating the X velocity & position
    Quad.X_dot = Quad.X_ddot*Quad.Ts + Quad.X_dot;
    Quad.X = Quad.X_dot*Quad.Ts + Quad.X;

    % Calculating the Y velocity & position
    Quad.Y_dot = Quad.Y_ddot*Quad.Ts + Quad.Y_dot;
    Quad.Y = Quad.Y_dot*Quad.Ts + Quad.Y;

    % Calculating p,q,r
    Quad.p = Quad.p_dot*Quad.Ts+Quad.p;
    Quad.q = Quad.q_dot*Quad.Ts+Quad.q;
    Quad.r = Quad.r_dot*Quad.Ts+Quad.r;

    % Calculating angular velocity and position
    Quad.phi = Quad.phi_dot*Quad.Ts + Quad.phi;
    Quad.theta = Quad.theta_dot*Quad.Ts+Quad.theta;
    Quad.psi = Quad.psi_dot*Quad.Ts+Quad.psi;

    %% Update Plotting Variables

    % Flip positive Z axis up for intuitive plotting
    Quad.Z_plot(Quad.counter) = -Quad.Z;
    Quad.Z_ref_plot(Quad.counter) = -Quad.Z_des;
    Quad.Z_dot_plot(Quad.counter) = -Quad.Z_dot;

    Quad.X_plot(Quad.counter) = Quad.X;
    Quad.X_ref_plot(Quad.counter) = Quad.X_des;
    Quad.X_dot_plot(Quad.counter) = Quad.X_dot;

    Quad.Y_plot(Quad.counter) = Quad.Y;
    Quad.Y_ref_plot(Quad.counter) = Quad.Y_des;
    Quad.Y_dot_plot(Quad.counter) = Quad.Y_dot;

    Quad.phi_plot(Quad.counter) = Quad.phi;
    Quad.phi_ref_plot(Quad.counter) = Quad.phi_des;

    Quad.theta_plot(Quad.counter) = Quad.theta;
    Quad.theta_ref_plot(Quad.counter) = Quad.theta_des;

    Quad.psi_plot(Quad.counter) = Quad.psi;
    Quad.psi_ref_plot(Quad.counter) = Quad.psi_des;
    
    % Absolute XY velocity
    Quad.Abs_Vel_plot(Quad.counter) = sqrt(Quad.X_dot^2 + Quad.Y_dot^2);
    
    Quad.gamma_plot(Quad.counter) = Quad.gamma;

    Quad.counter = Quad.counter + 1;

end

