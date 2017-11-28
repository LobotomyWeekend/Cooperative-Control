function [Quad] = quad_variables(sim)
    %% Initialize Quadrotor Variables
    % simulation parameters
    Quad.init = 0;     % used in initilization 
    Quad.counter = 1; 

    % plotting variables
    Quad.Ts = sim.Ts; 
    Quad.sim_time = sim.Tend;
    Quad.t_plot = [0:Quad.Ts:Quad.sim_time-Quad.Ts];

    % coordination
    Quad.gamma = 0;

    % Environmental Parametes
    Quad.g = 9.81;     % gravity (m/s^2)

    % Quadrotor Physical Parameters
    Quad.m = 1.4;      % mass (kg)
    Quad.l = .56;     % distance from center of mass to the each motor (m)
    Quad.Kd = 1.3858e-6;    % drag torque coeffecient (kg-m^2)

    Quad.Kdx = 0.16481;    % translational drag force coeffecient (kg/s)
    Quad.Kdy = 0.31892;    % translational drag force coeffecient (kg/s)
    Quad.Kdz = 1.1E-6;    % translational drag force coeffecient (kg/s)

    Quad.Jx = .05;     % moment of inertia about X axis (kg-m^2)
    Quad.Jy = .05;     % moment of inertia about Y axis (kg-m^2)
    Quad.Jz = .24;    % moment of inertia about Z axis (kg-m^2)

    % Motor Parameters
    Quad.KT = 1.3328e-5;    % thrust force coeffecient (kg-m)
    Quad.Jp = 0.044;     % moment of intertia of the rotor (kg-m^2)
    Quad.max_motor_speed = 925; % motors upper limit (rad/s)
    Quad.min_motor_speed = 0; %-1*((400)^2); % motors lower limit (can't spin in reverse)

    Quad.Obar = 0;     % sum of motor speeds (O1-O2+O3-O4, N-m) 
    Quad.O1 = 0;       % Front motor speed (raidans/s)
    Quad.O2 = 0;       % Right motor speed (raidans/s)
    Quad.O3 = 0;       % Rear motor speed (raidans/s)
    Quad.O4 = 0;       % Left motor speed (raidans/s)

    %Translational Positions
    Quad.X = 0;        % initial position in X direction GF (m)
    Quad.Y = 0;        % initial position in Y direction GF (m)
    Quad.Z = 0;        % initial position in Z direction GF (m)
    Quad.X_BF = 0;     % initial position in X direction BF (m)
    Quad.Y_BF = 0;     % initial position in Y direction BF (m)
    Quad.Z_BF = 0;     % initial position in the Z direction BF (m)

    %Translational Velocities
    Quad.X_dot = 0;    % initial velocity in X direction GF (m/s)
    Quad.Y_dot = 0;    % initial velocity in Y direction GF (m/s)
    Quad.Z_dot = 0;    % initial velocity in Z direction GF (m/s)
    Quad.X_dot_BF = 0;    % initial velocity in X direction BF (m/s)
    Quad.Y_dot_BF = 0;    % initial velocity in Y direction BF (m/s)
    Quad.Z_dot_BF = 0;    % initial velocity in Y direction BF (m/s)

    %Angular Positions
    Quad.phi = 0;      % initial phi value (rotation about X GF, roll,  radians)
    Quad.theta = 0;    % initial theta value (rotation about Y GF, pitch, radians)
    Quad.psi = 0;      % initial psi value (rotation about Z GF, yaw, radians)

    %Angular Velocities
    Quad.p = 0;        % initial p value (angular rate rotation about X BF, radians/s)
    Quad.q = 0;        % initial q value (angular rate rotation about Y BF, radians/s)
    Quad.r = 0;        % initial r value (angular rate rotation about Z BF, radians/s)

    % Desired variables
    Quad.X_des_GF = 1;         % desired value of X in Global frame
    Quad.Y_des_GF = 1;         % desired value of Y in Global frame
    Quad.Z_des_GF = 1;         % desired value of Z in Global frame
    Quad.X_des = 0;            % desired value of X in Body frame
    Quad.Y_des = 0;            % desired value of Y in Body frame
    Quad.Z_des = 0;            % desired value of Z in Body frame

    Quad.phi_des = 0;          % desired value of phi (radians)
    Quad.theta_des = 0;        % desired value of theta (radians)
    Quad.psi_des = pi/6;          % desired value of psi (radians)

    % Disturbance Variables
    Quad.Z_dis = 0;            % disturbance in Z direction
    Quad.X_dis = 0;            % disturbance in X direction
    Quad.Y_dis = 0;            % disturbance in Y direction
    Quad.phi_dis = 0;            % disturbance in Yaw direction
    Quad.theta_dis = 0;            % disturbance in Pitch direction
    Quad.psi_dis = 0;            % disturbance in Roll direction

    % Control Inputs
    Quad.U1 = 0;       % total thrust (N)
    Quad.U2 = 0;       % torque about X axis BF (N-m)
    Quad.U3 = 0;       % torque about Y axis BF (N-m)
    Quad.U4 = 0;       % torque about Z axis BF (N-m)

    % Control Limits (update values)
    Quad.U1_max = 43.5;   % Quad.KT*4*Quad.max_motor_speed^2
    Quad.U1_min = 0;      % 
    Quad.U2_max = 6.25;  % Quad.KT*Quad.l*Quad.max_motor_speed^2
    Quad.U2_min = -6.25; % Quad.KT*Quad.l*Quad.max_motor_speed^2
    Quad.U3_max = 6.25;  % Quad.KT*Quad.l*Quad.max_motor_speed^2
    Quad.U3_min = -6.25; % Quad.KT*Quad.l*Quad.max_motor_speed^2
    Quad.U4_max = 2.25; % Quad.Kd*2*Quad.max_motor_speed^2
    Quad.U4_min = -2.25;% Quad.Kd*2*Quad.max_motor_speed^2

    % PID parameters
    Quad.X_KP = .35;          % KP value in X position control
    Quad.X_KI = .25;            % KI value in X position control
    Quad.X_KD = -.35;         % KD value in X position control
    Quad.X_KI_lim = .25;         % Error to start calculating integral term

    Quad.Y_KP = .35;          % KP value in Y position control
    Quad.Y_KI = .25;            % KI value in Y position control
    Quad.Y_KD = -.35;         % KD value in Y position control
    Quad.Y_KI_lim = .25;         % Error to start calculating integral term

    Quad.Z_KP = 10/1.7;    % KP value in altitude control
    Quad.Z_KI = 0*3;    % KI value in altitude control
    Quad.Z_KD = -10/1.980;  % KD value in altitude control
    Quad.Z_KI_lim = .25;         % Error to start calculating integral term

    Quad.phi_KP = 4.5;      % KP value in roll control 2
    Quad.phi_KI = 0;       % KI value in roll control   1        
    Quad.phi_KD = 0;     % KD value in roll control  -.5
    Quad.phi_max = pi/4;   % Maximum roll angle commanded
    Quad.phi_KI_lim = 2*(2*pi/360);  % Error to start calculating integral 

    Quad.theta_KP = 4.5;    % KP value in pitch control 2
    Quad.theta_KI = 0;     % KI value in pitch control 1
    Quad.theta_KD = 0;   % KD value in pitch control -.5
    Quad.theta_max = pi/4; % Maximum pitch angle commanded
    Quad.theta_KI_lim = 2*(2*pi/360);  % Error to start calculating integral 

    Quad.psi_KP = 10;     % KP value in yaw control
    Quad.psi_KI = 0;     % KI value in yaw control .75
    Quad.psi_KD = 0;     % KD value in yaw control -.5
    Quad.psi_KI_lim = 8*(2*pi/360);  % Error to start calculating integral 

    Quad.p_KP = 2.7;    % KP value in pitch control 2
    Quad.p_KI = 1;     % KI value in pitch control
    Quad.p_KD = -.01;   % KD value in pitch control -.5
    Quad.p_max = 50*(2*pi/360); % Maximum pitch angle commanded
    Quad.p_KI_lim = 10*(2*pi/360);  % Error to start calculating integral 

    Quad.q_KP = 2.7;    % KP value in pitch control
    Quad.q_KI = 1;     % KI value in pitch control
    Quad.q_KD = -.01;   % KD value in pitch control -.5
    Quad.q_max = 50*(2*pi/360); % Maximum pitch angle commanded
    Quad.q_KI_lim = 10*(2*pi/360);  % Error to start calculating integral 

    Quad.r_KP = 2.7;    % KP value in pitch control
    Quad.r_KI = 1;     % KI value in pitch control
    Quad.r_KD = -.01;   % KD value in pitch control
    Quad.r_max = 50*(2*pi/360); % Maximum pitch angle commanded
    Quad.r_KI_lim = 10*(2*pi/360);  % Error to start calculating integral 
end
