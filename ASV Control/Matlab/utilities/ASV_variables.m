function ASV = ASV_variables(sim, initialPosition, initialYaw, vehicleID)
    %% Establishes all variables for the ASV struct variable
    % Sets all variables to zero unless initial conditions are applied,
    % constant coefficients are included for use in the vehicle dynamics
    % function.
    % Arguments initialPosition and initialYaw are applied so long as they are
    % provided.
    % VehicleID is useful in applying vcorr during coordination
    % simulations, and may also be used in the legends during plotting to
    % identify vehicles
    
    % basic vehicle properties
    ASV.vehicleType = "ASV";
    if nargin > 3
        ASV.vehicleID = vehicleID;
    else
        ASV.vehicleID = 1;
    end
    
    % time properties
    ASV.Ts   = sim.Ts;
    ASV.Tend = sim.Tend;
    ASV.time = sim.time;
    
    % path section for component paths
    ASV.section = 1; 

    % Translational Position
    
    
    % if initial position specified, apply
    if nargin > 1
        ASV.X = initialPosition(1,1);
        ASV.Y = initialPosition(2,1);
        ASV.Z = 0; % assumed
    else
    % if no initial position provided, locate vehicle on origin of {G}
    % frame
        ASV.X = 0;          % position in X ( GF )
        ASV.Y = 0;          % position in Y ( GF )
        ASV.Z = 0;          % position in Z ( GF )
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
    ASV.Roll = 0;           % rotation about X {G}
    ASV.Pitch = 0;          % rotation about Y {G}
    ASV.Yaw = initialYaw;   % rotation about Z {G}
    
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
    
    % Vehicle Properties, constant coefficients
    ASV.m       = 17;           % mass (kg)
    ASV.Iz      = 1;            % moment of inertia about z kg m^2
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
    ASV.gamma     = 0;          % initial coordination state = 0
    ASV.vcorr = 0;
    
    % Error variables
    ASV.error_crossTrack = 0;   % cross track error (m)
    ASV.error_yaw = 0;          % error in yaw
    ASV.gamma_err = 0;          % coordination errror

    % Integral hold values
    ASV.speed_int = 0;
    ASV.yaw_int   = 0;
    ASV.error_crossTrack_int = 0;
    
    % variables used in various initialisations
    ASV.latch = 0;              
    ASV.init = 0;
    
    % Counter value
    ASV.counter = 1;

end