%% Simulates UAV-UAV Cooperation
% Two UAVs working in coordination for any suitable path type, plots
% trajectory, cross track error, and coordination states over time.

%% Initialize Workspace
clear all;
close all;
clc;

% constants
complete = 1.0;

%% Simulation inputs
sim.Ts = 0.01;
sim.Tend = 360;
sim.t = 0:sim.Ts:sim.Tend;

%% Path Variables & References
% Vehicle 1
ref1.pathType = 1;
ref1.start = [0; 0];
ref1.finish = [20; 20];
ref1.uRefNominal = 0.5;
% Vehicle 2
ref2.pathType = 1;
ref2.start = [2; 0];
ref2.finish = [22; 20];
ref2.uRefNominal = 0.5;

%% Initialize Vehicles
% UAV1
UAV1 = quad_variables(sim, 1, ref1.start);
UAV1 = quad_dynamics_nonlinear(UAV1);
% UAV2
UAV2 = quad_variables(sim, 2, ref2.start);
UAV2 = quad_dynamics_nonlinear(UAV2);

%% Run The Simulation Loop
i = 1;
for t = sim.t
    %% Simulation
    % Master coordination controller
    vCorr = coordinationMaster(UAV1, UAV2);
    
    % Path Follower
    UAV1 = lookaheadPathFollowerUAV(UAV1, ref1, vCorr(UAV1.vehicleID));
    UAV2 = lookaheadPathFollowerUAV(UAV2, ref2, vCorr(UAV2.vehicleID));

    % End condition
    UAV1 = endConditionUAV(UAV1, ref1, complete);
    UAV2 = endConditionUAV(UAV2, ref2, complete);
    
    % Inner Loop Dynamics and Controllers
    UAV1 = innerLoopUAV(UAV1);
    UAV2 = innerLoopUAV(UAV2);
    
    %% Save data
    gammaHist(:,i) = [UAV1.gamma;UAV2.gamma];
    
    i = i + 1;
end

%% Plots
% trajectory
plotTrajectory(UAV1, UAV2);
% coordination state
plotCoordination(UAV1, UAV2);
% cross track error
plotCrossTrackError(UAV1, UAV2);
