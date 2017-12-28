function UAV = quad_dynamics_nonlinear(UAV)
    UAV.counter = UAV.counter + 1;

    %% Update Accelerations

    UAV.X_ddot = (-[cos(UAV.phi)*sin(UAV.theta)*cos(UAV.psi)+sin(UAV.phi)*sin(UAV.psi)]*UAV.U1-UAV.Kdx*UAV.X_dot)/UAV.m;
    UAV.Y_ddot = (-[cos(UAV.phi)*sin(UAV.psi)*sin(UAV.theta)-cos(UAV.psi)*sin(UAV.phi)]*UAV.U1-UAV.Kdy*UAV.Y_dot)/UAV.m;
    UAV.Z_ddot = (-[cos(UAV.phi)*cos(UAV.theta)]*UAV.U1-UAV.Kdz*UAV.Z_dot)/UAV.m+UAV.g;

    UAV.p_dot = (UAV.q*UAV.r*(UAV.Jy - UAV.Jz) - UAV.Jp*UAV.p*UAV.Obar + UAV.l*UAV.U2)/UAV.Jx;
    UAV.q_dot = (UAV.p*UAV.r*(UAV.Jz - UAV.Jx) + UAV.Jp*UAV.q*UAV.Obar + UAV.l*UAV.U3)/UAV.Jy;
    UAV.r_dot = (UAV.p*UAV.q*(UAV.Jx - UAV.Jy) + UAV.U4)/UAV.Jz;

    UAV.phi_dot   = UAV.p + sin(UAV.phi)*tan(UAV.theta)*UAV.q + cos(UAV.phi)*tan(UAV.theta)*UAV.r;
    UAV.theta_dot = cos(UAV.phi)*UAV.q - sin(UAV.phi)*UAV.r;
    UAV.psi_dot   = sin(UAV.phi)/cos(UAV.theta)*UAV.q + cos(UAV.phi)/cos(UAV.theta)*UAV.r;

    %% Disturbance model

    UAV.X_ddot = UAV.X_ddot + UAV.X_dis/UAV.m; 
    UAV.Y_ddot = UAV.Y_ddot + UAV.Y_dis/UAV.m; 
    UAV.Z_ddot = UAV.Z_ddot + UAV.Z_dis/UAV.m; 
    UAV.phi_dot = UAV.phi_dot + UAV.phi_dis/UAV.Jx*UAV.Ts; 
    UAV.theta_dot = UAV.theta_dot + UAV.theta_dis/UAV.Jy*UAV.Ts; 
    UAV.psi_dot = UAV.psi_dot + UAV.psi_dis/UAV.Jz*UAV.Ts;

    %% Update Velocities and Positions

    % Calculating the Z velocity & position
    UAV.Z_dot = UAV.Z_ddot*UAV.Ts + UAV.Z_dot;
    UAV.Z = UAV.Z_dot*UAV.Ts + UAV.Z;

    % Calculating the X velocity & position
    UAV.X_dot = UAV.X_ddot*UAV.Ts + UAV.X_dot;
    UAV.X = UAV.X_dot*UAV.Ts + UAV.X;

    % Calculating the Y velocity & position
    UAV.Y_dot = UAV.Y_ddot*UAV.Ts + UAV.Y_dot;
    UAV.Y = UAV.Y_dot*UAV.Ts + UAV.Y;

    % Calculating p,q,r
    UAV.p = UAV.p_dot*UAV.Ts+UAV.p;
    UAV.q = UAV.q_dot*UAV.Ts+UAV.q;
    UAV.r = UAV.r_dot*UAV.Ts+UAV.r;

    % Calculating angular velocity and position
    UAV.phi = UAV.phi_dot*UAV.Ts + UAV.phi;
    UAV.theta = UAV.theta_dot*UAV.Ts+UAV.theta;
    UAV.psi = UAV.psi_dot*UAV.Ts+UAV.psi;

    %% Update Plotting Variables

    % Flip positive Z axis up for intuitive plotting
    UAV.Z_plot(UAV.counter) = -UAV.Z;
    UAV.Z_ref_plot(UAV.counter) = -UAV.Z_des;
    UAV.Z_dot_plot(UAV.counter) = -UAV.Z_dot;

    UAV.X_plot(UAV.counter) = UAV.X;
    UAV.X_ref_plot(UAV.counter) = UAV.X_des;
    UAV.X_dot_plot(UAV.counter) = UAV.X_dot;

    UAV.Y_plot(UAV.counter) = UAV.Y;
    UAV.Y_ref_plot(UAV.counter) = UAV.Y_des;
    UAV.Y_dot_plot(UAV.counter) = UAV.Y_dot;

    UAV.phi_plot(UAV.counter) = UAV.phi;
    UAV.phi_ref_plot(UAV.counter) = UAV.phi_des;

    UAV.theta_plot(UAV.counter) = UAV.theta;
    UAV.theta_ref_plot(UAV.counter) = UAV.theta_des;

    UAV.psi_plot(UAV.counter) = UAV.psi;
    UAV.psi_ref_plot(UAV.counter) = UAV.psi_des;
    
    % Absolute XY velocity
    UAV.Abs_Vel_plot(UAV.counter) = sqrt(UAV.X_dot^2 + UAV.Y_dot^2);
    
    UAV.gamma_plot(UAV.counter) = UAV.gamma;
        

end

