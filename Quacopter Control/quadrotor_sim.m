%% Simulates quadrotor dynamics and implements a control algorithm
% This simulation is a new version 26/12/2017 which controls the UAV in
% absolute velocity and heading. This is the same method as with the ASV,
% and has a number of advantages over waypoint control.

clear all;
close all;
clc;

%% Initialize Workspace
% constants
complete = 1.0;
vCorr = 0.0;

%% Simulation inputs
sim.Ts = 0.01;
sim.Tend = 60;

%% Path Variables & References
ref.pathType = 2;
ref.start = [0; 0]; % also vehicle's initial position
ref.finish = [20; 0];
ref.uRefNominal = 0.5;
ref.uRef = 0.5;

ref.waypoints = ref_waypoints(ref);

%% Initialize Vehicle
UAV = quad_variables(sim,ref.start);
UAV.X = 0.1;
UAV = quad_dynamics_nonlinear(UAV);
UAV.ref = ref;

%% Run The Simulation Loop
for t = UAV.t_plot
    % Display Progression
    displayProgress(UAV);
    
    % Path Follower
    UAV = pathFollowerUAV(UAV, ref);

    % Inner Loop Dynamics and Controllers
    UAV = innerLoopUAV(UAV);
    
    % End condition 
    UAV = endConditionUAV(UAV);
end
clc
disp('Finishing Up...');

%% Plots
% trajectory
plotTrajectory(UAV);
% coordination state
plotCoordination(UAV);
% cross track error
plotCrossTrackError(UAV);
% control terms
% PlotUAVCommands(UAV);

%% TEST
figure('Name','Heading');
hold on
plot(UAV.t_plot, UAV.heading_ref_plot, '--');
plot(UAV.t_plot, UAV.heading_plot);
hold off

clc;