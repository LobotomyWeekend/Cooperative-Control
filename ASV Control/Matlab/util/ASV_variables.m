function ASV = ASV_variables(sim, initialPosition, initialYaw, vehicleID)
    %% Establishes all variables for the ASV struct variable
    % sets all variables to zero unless initial conditions are applied,
    % other properties are set as the vehicle properties
    
    % vehicle properties for cooperation
    ASV.vehicleType = "ASV";
    ASV.vehicleID = vehicleID;
    
    % time properties
    ASV.Ts   = sim.Ts;
    ASV.Tend = sim.Tend;
    ASV.time = sim.time;
    
    % simulation parameters
    Quad.section = 1; % path section for component paths

    % Translational Position
    ASV.X = 0;          % position in X ( GF )
    ASV.Y = 0;          % position in Y ( GF )
    ASV.Z = 0;          % position in Z ( GF )
    
    % same as above if initial position specified
    if nargin > 1
        ASV.X = initialPosition(1,1);
        ASV.Y = initialPosition(2,1);
        ASV.Z = 0; % assumed
    end
    
    % Translational Velocities
    ASV.X_dot = 0;      % velocity in X ( GF )
    ASV.Y_dot = 0;      % velocity in Y ( GF )
    ASV.Z_dot= 0;       % velocity in Z ( GF )
    
    % Translational Accelerations
    ASV.X_dot_dot = 0;  % acceleration in X ( GF )
    ASV.Y_dot_dot = 0;  % acceleration in Y ( GF )
    ASV.Z_dot_dot = 0;  % acceleration in Z ( GF )
    
    % Translational Velocities ( BF )
    ASV.u = 0;
    ASV.v = 0;
    ASV.w = 0;
    
    % Translational Accelerations ( BF )
    ASV.u_dot = 0;
    ASV.v_dot = 0;
    ASV.w_dot = 0;
        
    % Rotational Position
    ASV.Roll = 0;       % rotation about X
    ASV.Pitch = 0;      % rotation about Y
    ASV.Yaw = initialYaw;  % rotation about Z
    
    % Rotational Rates
    ASV.Roll_dot = 0;
    ASV.Pitch_dot = 0;
    ASV.Yaw_dot =0;
    
    % Rotation Rates
    ASV.p = 0;          % rotation rate about X
    ASV.q = 0;          % rotation rate about Y
    ASV.r = 0;          % rotation rate about Z

    % Rotational Acceleration
    ASV.p_dot = 0;      % acceleration in roll
    ASV.q_dot = 0;      % acceleration in pitch
    ASV.r_dot = 0;      % acceleration in yaw
    
    % Vehicle Properties
    ASV.m       = 17;         % mass (kg)
    ASV.Iz      = 1;
    ASV.X_u_dot = -20;
    ASV.Y_v_dot = -30;
    ASV.N_r_dot = -8.69;
    ASV.X_u     = -0.2;
    ASV.Y_v     = -50;
    ASV.N_r     = -4.14;
    ASV.X_uu    = -25;
    ASV.Y_vv    = -0.01;
    ASV.N_rr    = -6.23;

    % Coordination variables
    ASV.gamma     = 0;
    ASV.gamma_err = 0;
    ASV.vcorr = 0;
    
    % Error variables
    ASV.error_crossTrack = 0; % cross track error (m)
    ASV.error_yaw = 0;

    % Integral hold values
    ASV.speed_int = 0;
    ASV.yaw_int   = 0;
    ASV.error_crossTrack_int = 0;
    
    % Latch
    ASV.latch = 0;
    
    % Counter value
    ASV.counter = 1;

end