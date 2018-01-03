function UAV = speed_PID(UAV)

    persistent X_dot_error_sum;
    persistent Y_dot_error_sum;

    % initialize persistent variables at beginning of simulation
    if UAV.init == 0
        X_dot_error_sum = 0;
        Y_dot_error_sum = 0;
    end
    
    %% Extract Information
    % current orientation
    phi     = UAV.phi;      % roll angle {G}
    theta   = UAV.theta;    % pitch angle {G}
    psi     = UAV.psi;      % yaw angle {G}
    % current velocity & acceleration
    X_dot   = UAV.X_dot;    % X speed {G}
    X_ddot  = UAV.X_ddot;   % X acceleration {G}
    Y_dot   = UAV.Y_dot;    % Y speed {G}
    Y_ddot  = UAV.Y_ddot;   % Y acceleration {G}
    Z_dot   = UAV.Z_dot;    % Z speed {G}
    Z_ddot  = UAV.Z_ddot;   % Z acceleration {G}
    % desired velocities
    X_dot_GF_des = UAV.X_dot_GF_des;
    Y_dot_GF_des = UAV.Y_dot_GF_des;
    Z_dot_GF_des = UAV.Z_dot_GF_des;
    
    % transform current velocity {G} to {B}
    [X_dot_BF, Y_dot_BF, Z_dot_BF] = rotateGFtoBF(X_dot, Y_dot, Z_dot, phi, theta, psi);
    
    % transform desired velocity {G} to {B}
    [X_dot_BF_des, Y_dot_BF_des, Z_dot_BF_des] = rotateGFtoBF(X_dot_GF_des, Y_dot_GF_des, Z_dot_GF_des, phi, theta, psi);
    
    % transform current acceleration {G} to {B}
    [X_ddot_BF, Y_ddot_BF, Z_ddot_BF] = rotateGFtoBF(X_ddot, Y_ddot, Z_ddot, phi, theta, psi);
    
    %% Speed controller in X
    % error
    X_dot_error = X_dot_BF_des - X_dot_BF;
   
    % proportional
    cp = UAV.X_KP * X_dot_error;
    
    % integral
    X_dot_error_sum =  X_dot_error_sum + X_dot_error;
    ci = UAV.X_KI * UAV.Ts * X_dot_error_sum;
    
    % differential
    cd = UAV.X_KD * X_ddot_BF;
    
    % desired pitch
    UAV.theta_des =  - (cp + ci + cd);   %Theta and X inversely related
    UAV.theta_des = min(UAV.theta_max, max(-UAV.theta_max, UAV.theta_des));

    
    %% Speed controller in Y
    % error
    Y_dot_error = Y_dot_BF_des - Y_dot_BF;
   
    % proportional
    cp = UAV.Y_KP * Y_dot_error;
    
    % integral
    Y_dot_error_sum =  Y_dot_error_sum + Y_dot_error;
    ci = UAV.Y_KI * UAV.Ts * Y_dot_error_sum;
    
    % differential
    cd = UAV.Y_KD * Y_ddot_BF;
    
    % desired roll
    UAV.phi_des = cp + ci + cd; 
    UAV.phi_des = min(UAV.phi_max, max(-UAV.phi_max, UAV.phi_des));

    % Speed Reference Plots
    UAV.X_dot_GF_des_plot(UAV.counter) = UAV.X_dot_GF_des;
    UAV.Y_dot_GF_des_plot(UAV.counter) = UAV.Y_dot_GF_des;
end