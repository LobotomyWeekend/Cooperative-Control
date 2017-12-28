%% Coordination between 1 UAV + 1 ASV
% Test simulation for cooperative motion between the two types of vehicle,
% for any supported path type.
%% Initialize Workspace
clear all;
close all;
clc;

% constants
complete = 1.0;

%% Simulation inputs
sim.Ts = 0.01;
sim.Tend = 60;
sim.time = 0:sim.Ts:sim.Tend;

%% UAV Setup
% Vehicle 1
ref1.pathType = 1;
ref1.start = [0; 0];
ref1.finish = [20; 20];
% constant speed reference
ref1.uRefNominal = 0.5;
% initialize vehicle structure
UAV1 = quad_variables(sim, 1, ref1.start);
UAV1 = quad_dynamics_nonlinear(UAV1);
UAV1.ref = ref1;

%% ASV Setup
% waypoints
ref2.pathType = 1;
ref2.start = [5;0];
ref2.finish = [25;20];
% constant speed reference
ref2.uRefNominal = 0.5;
% initial values
yawInit = 45;
% initialize vehicle structure
ASV2 = ASV_variables(sim, ref2.start, yawInit, 2);
ASV2.ref = ref2;

%% Constants (SIMPLIFIED SIMULATION)
vCorr = [0;0];
ref2.uRef = ref2.uRefNominal;


%% SIMULATION
for t =  sim.time
    % Display Progression
    displayProgress(UAV1);
    
    % Coordination
    vCorr = coordinationMaster(UAV1, ASV2);
    
    % Update Speed Reference
    UAV1.ref.uRef = ref1.uRefNominal + vCorr(1);
    ref2.uRef = ref2.uRefNominal + vCorr(ASV2.vehicleID);

    % Path Following
    UAV1 = pathFollowerUAV(UAV1, ref1);
    [ref2.yawRef, ASV2] = pathFollowerASV(ASV2, ref2);
    
%     % End Condition
%     UAV1 = endConditionUAV(UAV1);
%     [ref2, ASV2] = endConditionASV(ASV2, ref2, complete);

    % Simulate Vehicles
    UAV1 = innerLoopUAV(UAV1);
    ASV2 = innerLoopASV(ref2, ASV2);
    
end
clc
disp('Finishing Up...');

%% Plotting
% trajectory
plotTrajectory(UAV1, ASV2);
% coordination
plotCoordination(UAV1, ASV2);
% cross track error
plotCrossTrackError(UAV1, ASV2);