%% Finding ASV minimum arc diameter
% The actuators of the vehicle can only achieve a certain turn rate, which 

clear all;
close all;
clc;

complete = 1;
section = 0;

%% Simulation Inputs
% time
sim.Ts   = 0.1;
sim.Tend = 150;
sim.time = 0:sim.Ts:sim.Tend;

dmax = 20;
inc = 5/2; % incremental radius change

% reference vehicle 1 [d(arc) = 20]
ref.pathType = 2;
ref.start = [0;0];
ref.finish = [dmax;0];
ref.uRef = 0.5;
ref.uRefNominal = 0.5;

% reference vehicle 2 [d(arc) = 75]
ref2.pathType = 2;
ref2.start = [inc;0];
ref2.finish = [dmax - inc;0];
ref2.uRef = 0.5;
ref2.uRefNominal = 0.5;

% reference vehicle 3 [d(arc) = 50]
ref3.pathType = 2;
ref3.start = [2*inc;0];
ref3.finish = [dmax - 2*inc;0];
ref3.uRef = 0.5;
ref3.uRefNominal = 0.5;

% reference vehicle 4 [d(arc) = 25]
ref4.pathType = 2;
ref4.start = [3*inc; 0];
ref4.finish = [dmax - 3*inc; 0];
ref4.uRef = 0.5;
ref4.uRefNominal = 0.5;

% initial yaw value
yawInit = 90; 

%% Initialize Vehicles
% establish structure
ASV1 = ASV_variables(sim, ref.start, yawInit, 1);
ASV2 = ASV_variables(sim, ref2.start, yawInit, 2);
ASV3 = ASV_variables(sim, ref3.start, yawInit, 3);
ASV4 = ASV_variables(sim, ref4.start, yawInit, 4);

%% Simulation
for t = sim.time    
    %% Calculate References
    [ref.yawRef, ASV1] = pathFollowerASV(ASV1, ref);      
    [ref2.yawRef, ASV2] = pathFollowerASV(ASV2, ref2); 
    [ref3.yawRef, ASV3] = pathFollowerASV(ASV3, ref3); 
    [ref4.yawRef, ASV4] = pathFollowerASV(ASV4, ref4); 
    
    %% Simulate Vehicles
    % ASV 1
    [ASV1] = innerLoopASV(ref, ASV1);
    % stick to end for clarity
    if ASV1.gamma >=1
        ASV1.X = ref.finish(1,1);
        ASV1.Y = ref.finish(2,1);
    end
    % ASV 2
    [ASV2] = innerLoopASV(ref2, ASV2);
    % stick to end for clarity
    if ASV2.gamma >=1
        ASV2.X = ref2.finish(1,1);
        ASV2.Y = ref2.finish(2,1);
    end
    % ASV 3
    [ASV3] = innerLoopASV(ref3, ASV3);
    % stick to end for clarity
    if ASV3.gamma >=1
        ASV3.X = ref3.finish(1,1);
        ASV3.Y = ref3.finish(2,1);
    end
    % ASV 4
    [ASV4] = innerLoopASV(ref4, ASV4);
    % stick to end for clarity
    if ASV4.gamma >=1
        ASV4.X = ref4.finish(1,1);
        ASV4.Y = ref4.finish(2,1);
    end
    
    displayProgress(ASV1);

end

%% Plotting
ASV1.ref = ref;
ASV2.ref = ref2;
ASV3.ref = ref3;
ASV4.ref = ref4;

plotTrajectory(ASV1, ASV2, ASV3, ASV4);
plotCrossTrackError(ASV1,ASV2,ASV3,ASV4);