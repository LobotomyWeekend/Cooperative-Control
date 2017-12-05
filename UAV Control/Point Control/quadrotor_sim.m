
%% Simulates quadrotor dynamics and implements a control algorithm
% Add Paths
mkdir('utilities')
addpath(genpath('utilities'))

%% Initialize Workspace
clear all;
close all;
clc;

% constants
complete = 1.0;
vCorr = 0.0;

%% Simulation inputs
sim.Ts = 0.01;
sim.Tend = 360;

%% Path Variables & References
ref.pathType = 1;
ref.start = [0; 0]; % also vehicle's initial position
ref.finish = [20; 20];
ref.uRefNominal = 0.5;

%% Initialize Vehicle
UAV = quad_variables(sim,ref.start);
UAV = quad_dynamics_nonlinear(UAV);

%% Run The Simulation Loop
for t = UAV.t_plot
    
    % Path Follower
    UAV = lookaheadPathFollowerUAV(UAV, ref);
    
    % Coordination
    UAV = coordinationUAV(UAV, vCorr, ref);
    
    % End condition
    UAV = endConditionUAV(UAV, ref, complete);
    
    % Inner Loop Dynamics and Controllers
    UAV = innerLoopUAV(UAV);
end

%% Plots
% trajectory
plotTrajectory(UAV);
% coordination state
plotCoordination(UAV);
% cross track error
plotCrossTrackError(UAV);
