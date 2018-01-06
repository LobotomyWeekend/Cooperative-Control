function [UAV] = quad_variables(sim, vehicleID, initialPosition)
    %% Initialize Quadrotor Variables
    % vehicle parameters
    UAV.vehicleType = "UAV";       % vehicle type UAV/ASV 
    if nargin > 1
        UAV.vehicleID = vehicleID; % vehicle ID [1,2,...,n]
    else 
        UAV.vehicleID = 1;         % standard vehicle ID
    end
    
    % simulation parameters
    UAV.init = 0;     % used in initilization 
    UAV.init_position_PID = 0;
    UAV.counter = 0; 
    UAV.section = 1; % path section for component paths
    UAV.complete = 0;

    % plotting variables
    UAV.Ts = sim.Ts; 
    UAV.sim_time = sim.Tend;
    UAV.t_plot = [0:UAV.Ts:UAV.sim_time-UAV.Ts];
    UAV.time = 0:UAV.Ts:UAV.sim_time;

    % coordination
    UAV.gamma = 0;

    % Environmental Parametes
    UAV.g = 9.81;     % gravity (m/s^2)

    % Quadrotor Physical Parameters
    UAV.m = 1.4;      % mass (kg)
    UAV.l = .56;     % distance from center of mass to the each motor (m)
    UAV.Kd = 1.3858e-6;    % drag torque coeffecient (kg-m^2)

    UAV.Kdx = 0.16481;    % translational drag force coeffecient (kg/s)
    UAV.Kdy = 0.31892;    % translational drag force coeffecient (kg/s)
    UAV.Kdz = 1.1E-6;    % translational drag force coeffecient (kg/s)

    UAV.Jx = .05;     % moment of inertia about X axis (kg-m^2)
    UAV.Jy = .05;     % moment of inertia about Y axis (kg-m^2)
    UAV.Jz = .24;    % moment of inertia about Z axis (kg-m^2)

    % Motor Parameters
    UAV.KT = 1.3328e-5;    % thrust force coeffecient (kg-m)
    UAV.Jp = 0.044;     % moment of intertia of the rotor (kg-m^2)
    UAV.max_motor_speed = 925; % motors upper limit (rad/s)
    UAV.min_motor_speed = 0; %-1*((400)^2); % motors lower limit (can't spin in reverse)

    UAV.Obar = 0;     % sum of motor speeds (O1-O2+O3-O4, N-m) 
    UAV.O1 = 0;       % Front motor speed (raidans/s)
    UAV.O2 = 0;       % Right motor speed (raidans/s)
    UAV.O3 = 0;       % Rear motor speed (raidans/s)
    UAV.O4 = 0;       % Left motor speed (raidans/s)

    %Translational Positions
    if nargin > 2
        % given initial position
        UAV.X = initialPosition(1,1);        % initial position in X direction GF (m)
        UAV.Y = initialPosition(2,1);        % initial position in Y direction GF (m)
    else 
        % otherwise, assume [0,0]
        UAV.X = 0;                 % initial position in X direction GF (m)
        UAV.Y = 0;                 % initial position in Y direction GF (m)
    end
    UAV.Z = 0;        % initial position in Z direction GF (m)
    UAV.X_BF = 0;     % initial position in X direction BF (m)
    UAV.Y_BF = 0;     % initial position in Y direction BF (m)
    UAV.Z_BF = 0;     % initial position in the Z direction BF (m)

    %Translational Velocities
    UAV.X_dot = 0;    % initial velocity in X direction GF (m/s)
    UAV.Y_dot = 0;    % initial velocity in Y direction GF (m/s)
    UAV.Z_dot = 0;    % initial velocity in Z direction GF (m/s)
    UAV.X_dot_BF = 0;    % initial velocity in X direction BF (m/s)
    UAV.Y_dot_BF = 0;    % initial velocity in Y direction BF (m/s)
    UAV.Z_dot_BF = 0;    % initial velocity in Y direction BF (m/s)

    %Angular Positions
    UAV.phi = 0;      % initial phi value (rotation about X GF, roll,  radians)
    UAV.theta = 0;    % initial theta value (rotation about Y GF, pitch, radians)
    UAV.psi = 0;      % initial psi value (rotation about Z GF, yaw, radians)

    %Angular Velocities
    UAV.p = 0;        % initial p value (angular rate rotation about X BF, radians/s)
    UAV.q = 0;        % initial q value (angular rate rotation about Y BF, radians/s)
    UAV.r = 0;        % initial r value (angular rate rotation about Z BF, radians/s)

    % Desired variables
    UAV.X_des_GF = 1;         % desired value of X in Global frame
    UAV.Y_des_GF = 1;         % desired value of Y in Global frame
    UAV.Z_des_GF = 1;         % desired value of Z in Global frame
    UAV.X_des = 0;            % desired value of X in Body frame
    UAV.Y_des = 0;            % desired value of Y in Body frame
    UAV.Z_des = 0;            % desired value of Z in Body frame

    UAV.phi_des = 0;          % desired value of phi (radians)
    UAV.theta_des = 0;        % desired value of theta (radians)
    UAV.psi_des = pi/6;       % desired value of psi (radians)
    
    UAV.yaw_desired = 0;      % desired value of yaw in GF (degrees)

    % Disturbance Variables
    UAV.Z_dis = 0.0;            % disturbance in Z direction
    UAV.X_dis = 0.0;            % disturbance in X direction
    UAV.Y_dis = 0.0;            % disturbance in Y direction
    UAV.phi_dis = 0;            % disturbance in Yaw direction
    UAV.theta_dis = 0;            % disturbance in Pitch direction
    UAV.psi_dis = 0;            % disturbance in Roll direction

    % Control Inputs
    UAV.U1 = 0;       % total thrust (N)
    UAV.U2 = 0;       % torque about X axis BF (N-m)
    UAV.U3 = 0;       % torque about Y axis BF (N-m)
    UAV.U4 = 0;       % torque about Z axis BF (N-m)

    % Control Limits (update values)
    UAV.U1_max = 43.5;   % Quad.KT*4*Quad.max_motor_speed^2
    UAV.U1_min = 0;      % 
    UAV.U2_max = 6.25;  % Quad.KT*Quad.l*Quad.max_motor_speed^2
    UAV.U2_min = -6.25; % Quad.KT*Quad.l*Quad.max_motor_speed^2
    UAV.U3_max = 6.25;  % Quad.KT*Quad.l*Quad.max_motor_speed^2
    UAV.U3_min = -6.25; % Quad.KT*Quad.l*Quad.max_motor_speed^2
    UAV.U4_max = 2.25; % Quad.Kd*2*Quad.max_motor_speed^2
    UAV.U4_min = -2.25;% Quad.Kd*2*Quad.max_motor_speed^2

    %% Gain Values for PID Controllers
    % KP : potential gain
    % KI : integral gain
    % KD : differential gain
    
    % Speed Control in X_dot
    UAV.X_dot_KP = 2.5;
    UAV.X_dot_KI = 0.5;
    UAV.X_dot_KD = -0.10;
    UAV.X_dot_KI_lim = 0.0; 

    % Speed Control in Y_dot
    % Set equal to above for convenience
    UAV.Y_dot_KP = UAV.X_dot_KP;
    UAV.Y_dot_KI = UAV.X_dot_KI;
    UAV.Y_dot_KD = UAV.X_dot_KD;
    UAV.Y_dot_KI_lim = UAV.X_dot_KI_lim;
    
    % Position Control in X
    UAV.X_KP = 0.35;
    UAV.X_KI = 0.25;
    UAV.X_KD = -0.35;
    UAV.X_KI_lim = 0;
    
    % Position Control in Y 
    % Set equal to above for convenience
    UAV.Y_KP = UAV.X_KP;
    UAV.Y_KI = UAV.X_KI;
    UAV.Y_KD = UAV.X_KD;
    UAV.Y_KI_lim = UAV.X_KI_lim;
    
    % Altitude Control in Z
    UAV.Z_KP = 10/1.7;
    UAV.Z_KI = 0.3;
    UAV.Z_KD = -10/1.980;
    UAV.Z_KI_lim = .25;

    % Attitude Control in phi (roll)
    UAV.phi_KP = 4.5;
    UAV.phi_KI = 0;      
    UAV.phi_KD = 0;
    UAV.phi_max = pi/4;
    UAV.phi_KI_lim = 2*(2*pi/360);

    % Attitude Control in theta (pitch)
    UAV.theta_KP = 4.5;
    UAV.theta_KI = 0;
    UAV.theta_KD = 0;
    UAV.theta_max = pi/4;
    UAV.theta_KI_lim = 2*(2*pi/360);

    % Attitude Control in psi (yaw)
    UAV.psi_KP = 10;
    UAV.psi_KI = 0;
    UAV.psi_KD = 0;
    UAV.psi_KI_lim = 8*(2*pi/360);

    % Rotation Rate Control in p (roll)
    UAV.p_KP = 2.7;
    UAV.p_KI = 1;
    UAV.p_KD = -.01;
    UAV.p_max = 50*(2*pi/360);
    UAV.p_KI_lim = 10*(2*pi/360);

    % Rotation Rate Control in q (pitch)
    UAV.q_KP = 2.7;
    UAV.q_KI = 1;
    UAV.q_KD = -.01;
    UAV.q_max = 50*(2*pi/360);
    UAV.q_KI_lim = 10*(2*pi/360);

    % Rotation Rate Control in r (yaw)
    UAV.r_KP = 2.7;
    UAV.r_KI = 1;
    UAV.r_KD = -.01;
    UAV.r_max = 50*(2*pi/360);
    UAV.r_KI_lim = 10*(2*pi/360);
end
