%% TESTING ASV INNER LOOP
% function which mimics the behaviour of the simulink model
% i.e. can be placed inside a time loop with ref values changed
% dynamically.
clear all;
close all;

%% Simulation Inputs
% time
sim.Ts   = 0.1;
sim.Tend = 500;
sim.time = 0:sim.Ts:sim.Tend;

%% Initialize Vehicles
%% VEHICLE 1
% waypoints
ref1.start = [0;0];
ref1.finish = [100;0];
% path type (line/arc)
ref1.pathType = 2; 
% initialising
yawInit = 45;
ref1.yawRef = yawInit;
ref1.uRef = 0.5;
ref1.uRefNominal = ref1.uRef;
% ASV struct
ASV1 = initializeASV(ref1, sim);
% initial conditions
[ASV1.IC, ASV.state] = initialConditions(ref1, yawInit);

%% VEHICLE 2
% waypoints
ref2.start = [10;0];
ref2.finish = [90;0];
% path type (line/arc)
ref2.pathType = 2; 
% initialising
yawInit = 45;
ref2.yawRef = yawInit;
ref2.uRef = 1;
ref2.uRefNominal = ref2.uRef;
% ASV struct
ASV2 = initializeASV(ref2, sim);
% initial conditions
[ASV2.IC, ASV.state] = initialConditions(ref2, yawInit);


%% Simulation
i = 1;
for t = sim.time
    %% Calculate Path Following References
    % ASV 1
    [ref1.yawRef, ASV1] = pathFollowerASV(ASV1, ref1, sim, i);
    % ASV 2
    [ref2.yawRef, ASV2] = pathFollowerASV(ASV2, ref2, sim, i);
    
    %% Calculate Coordination Correction
    [ref1.uRef, ref2.uRef, ASV1, ASV2] = coordination_2ASV(ASV1, ASV2, ref1, ref2);
    
    %% Simulate Vehicles
    % ASV 1
    [ASV1] = innerLoopASV(ref1, ASV1, sim, i);
    % ASV 2
    [ASV2] = innerLoopASV(ref2, ASV2, sim, i);

    %% Save Data
    % ASV 1
    ASV1.stateHist(i) = ASV1.state;
    ASV1.errorHist(i) = ASV1.error;
    ref1.refHist.yawRef(i) = ref1.yawRef;
    ref1.refHist.uRef(i) = ref1.uRef;
    ASV1.coordHist(i) = ASV1.coOrd;
    
    % ASV 1
    ASV2.stateHist(i) = ASV2.state;
    ASV2.errorHist(i) = ASV2.error;
    ref2.refHist.yawRef(i) = ref2.yawRef;
    ref2.refHist.uRef(i) = ref2.uRef;
    ASV2.coordHist(i) = ASV2.coOrd;

    
    i = i + 1;

end

%% Plots
% References and response
plotRefValues(ASV1, ref1, sim);
plotRefValues(ASV2, ref2, sim);

% Trajectory
plotTrajectory(ASV1, ref1, ASV2, ref2);

% Error
% plotErrorValues(ASV1, sim);
% plotErrorValues(ASV2, sim);

% Internal command (useful for troubleshooting)
% plotInternalCommands(ASV1,sim);

% Coordination States
plotCoordinationError(ASV1,ASV2, sim);

