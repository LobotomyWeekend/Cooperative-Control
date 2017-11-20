%% TESTING ASV INNER LOOP
% function which mimics the behaviour of the simulink model
% i.e. can be placed inside a time loop with ref values changed
% dynamically.
clear all;
close all;

%% Simulation Inputs
% time
sim.Ts   = 0.1;
sim.Tend = 220;
sim.time = 0:sim.Ts:sim.Tend;

% 1 = line, 2 = arc
ref.pathType = 1; 

% waypoints
ref.start = [0;0];
ref.finish = [20;0];

%% Initialize Vehicles
% vehicle 1
yawInit = 0;
ref.yawRef = yawInit;
% ASV sctruct
ASV1 = initializeASV(ref, sim);
% initial conditions
[ASV1.IC, ASV.state] = initialConditions(ref, yawInit);


%% Simulation
i = 1;
for t = sim.time
    %% Calculate References
    ref.uRef = 1;
   [ref.yawRef, ASV1] = pathFollowerASV(ASV1, ref, sim, i);
   
   %% Simulate Vehicles
    % ASV 1
    [ASV1] = innerLoopASV(ref, ASV1, sim, i);

    %% Save Data
    % state history
    ASV1.stateHist(i) = ASV1.state;
    ASV1.errorHist(i) = ASV1.error;
    ref.refHist.yawRef(i) = ref.yawRef;
    ref.refHist.uRef(i) = ref.uRef;
    i = i + 1;

end

%% Plots
% references and response
plotRefValues(ASV1, ref, sim);
% trajectory
plotTrajectory(ASV1, ref);
% error
plotErrorValues(ASV1, sim);
% internal command (useful for troubleshooting)
% plotInternalCommands(ASV1,sim);

