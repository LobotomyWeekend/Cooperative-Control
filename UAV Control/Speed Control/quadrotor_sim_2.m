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
ref1.pathType = 2;
ref1.start = [0; 0];
ref1.finish = [20; 0];
ref1.uRefNominal = 0.5;
ref1.waypoints = ref_waypoints(ref1);
% Vehicle 2
ref2.pathType = 2;
ref2.start = [2; 0];
ref2.finish = [18; 0];
ref2.uRefNominal = 0.5;
ref2.waypoints = ref_waypoints(ref2);

%% Initialize Vehicles
% UAV1
UAV1 = quad_variables(sim, 1, ref1.start);
UAV1 = quad_dynamics_nonlinear(UAV1);
% UAV2
UAV2 = quad_variables(sim, 2, ref2.start);
UAV2 = quad_dynamics_nonlinear(UAV2);

UAV1.ref = ref1;
UAV2.ref = ref2;

%% Run The Simulation Loop
for t = sim.t
    %% Simulation
    % Display Progression
    displayProgress(UAV1);
    
    % Master coordination controller
    vCorr = coordinationMaster(UAV1, UAV2);
    
    % Update speed reference
    UAV1.ref.uRef = ref1.uRefNominal + vCorr(1,1);
    UAV2.ref.uRef = ref2.uRefNominal + vCorr(1,2);
    
    % Path Follower
    UAV1 = pathFollowerUAV(UAV1, ref1);
    UAV2 = pathFollowerUAV(UAV2, ref2);
    
    % Inner Loop Dynamics and Controllers
    UAV1 = innerLoopUAV(UAV1);
    UAV2 = innerLoopUAV(UAV2);
    
    % End condition
    UAV1 = endConditionUAV(UAV1);
    UAV2 = endConditionUAV(UAV2);
    
end
clc
disp('Finishing Up...');

%% Plots
% trajectory
plotTrajectory(UAV1, UAV2);
% coordination state
plotCoordination(UAV1, UAV2);
% cross track error
plotCrossTrackError(UAV1, UAV2);

clc