%% Simulates UAV-UAV-UAV Cooperation
% Three UAVs working in coordination for any suitable path type, plots
% trajectory, cross track error, and coordination states over time.

%% Initialize Workspace
close all;
clc;

% constants
complete = 1.0;

%% Simulation inputs
sim.Ts = 0.01;
sim.Tend = 120;
sim.t = 0:sim.Ts:sim.Tend;

%% Path Variables & References
% Vehicle 1
ref1.pathType = 2;
ref1.start = [0; 0];
ref1.finish = [40; 0];
ref1.uRefNominal = 1;

% Vehicle 2
ref2.pathType = 2;
ref2.start = [5; 0];
ref2.finish = [35; 0];
ref2.uRefNominal = 1;

% Vehicle 3
ref3.pathType = 2;
ref3.start = [10; 0];
ref3.finish = [30; 0];
ref3.uRefNominal = 1;

%% Initialize Vehicles
% UAV1
UAV1 = quad_variables(sim, 1, ref1.start);
UAV1 = quad_dynamics_nonlinear(UAV1);

% UAV2
UAV2 = quad_variables(sim, 2, ref2.start);
UAV2 = quad_dynamics_nonlinear(UAV2);

% UAV 3
UAV3 = quad_variables(sim, 3, ref3.start);
UAV3 = quad_dynamics_nonlinear(UAV3);

%% Run The Simulation Loop
for t = sim.t
    %% Simulation
    % Master coordination controller
    vCorr = coordinationMaster(UAV1, UAV2, UAV3);
    
    % Path Follower
    UAV1 = lookaheadPathFollowerUAV(UAV1, ref1, vCorr(UAV1.vehicleID));
    UAV2 = lookaheadPathFollowerUAV(UAV2, ref2, vCorr(UAV2.vehicleID));
    UAV3 = lookaheadPathFollowerUAV(UAV3, ref3, vCorr(UAV3.vehicleID));

    % End condition
    UAV1 = endConditionUAV(UAV1, ref1, complete);
    UAV2 = endConditionUAV(UAV2, ref2, complete);
    UAV3 = endConditionUAV(UAV3, ref3, complete);
    
    % Inner Loop Dynamics and Controllers
    UAV1 = innerLoopUAV(UAV1);
    UAV2 = innerLoopUAV(UAV2);
    UAV3 = innerLoopUAV(UAV3);
    
    displayProgress(UAV1); % note: vehicle in argument arbitrary
    
end

%% Plots
% trajectory
plotTrajectory(UAV1, UAV2, UAV3);
% coordination state
plotCoordination(UAV1, UAV2, UAV3);
% cross track error
plotCrossTrackError(UAV1, UAV2, UAV3);
