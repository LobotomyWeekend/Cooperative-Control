%% Simulates quadrotor dynamics and implements a control algorithm
% This simulation is a new version 26/12/2017 which controls the UAV in
% absolute velocity and heading. This is the same method as with the ASV,
% and has a number of advantages over waypoint control.

clear all;
close all;
clc;

%% Simulation time
sim.Ts = 0.01;
sim.Tend = 20;s
sim.time = 0:sim.Ts:sim.Tend;

%% Initialize Vehicle
start = [0;0];
UAV = quad_variables(sim,start);
UAV.X_des_GF = 0;
UAV.Y_des_GF = 0;
UAV = quad_dynamics_nonlinear(UAV);

%% Run The Simulation Loop
i = 1;
for t = UAV.t_plot
    % Display Progression
    displayProgress(UAV);
    
    %% Provide Reference
    % provide positions ref
    if t > 5
        UAV.X_des_GF = 1;
        UAV.Y_des_GF = 1;
    end
    if t > 10
        UAV.Y_des_GF = 2;
    end
    %% Inner Loop
    % Position control
    UAV = position_PID(UAV);
    % Attitude and Rotation Rate Control
    UAV = attitude_PID(UAV);
    UAV = rate_PID(UAV);
    % Desired Motor Speeds
    UAV = quad_motor_speed(UAV);
    % Vehicle Model
    UAV = quad_dynamics_nonlinear(UAV); 
    
    UAV.init = 1; 
        
    i = i + 1;
end
clc
disp('Finishing Up...');

%% Plots
% % attitude reference and response
% plotAttitude(UAV);
% % rot rate reference and response
% plotRotationRate(UAV)
% position referece and repsonse
plotPosition(UAV);

clc;