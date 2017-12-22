%% COORDINATION ASV-ASV
% simulation of two ASV vehicles working in cooperation
clear all;
close all;
clc;

%% Simulation Inputs
% time
sim.Ts   = 0.1;
sim.Tend = 60*20;
sim.time = 0:sim.Ts:sim.Tend;

% waypoints
length_line = 100;
diameter_arc_min = 25;
offset = 5;
segments = 17;
no_vehicles = 2;

% lawnmower path for 3 vehicles
waypoints = waypointsMultiLawnmower(length_line, diameter_arc_min, offset, segments, no_vehicles);

% each vehicle's waypoints
waypoints_1 = waypoints(1:3,:);
waypoints_2 = waypoints(4:6,:);

% initial reference
ref1 = initialRef(waypoints_1);
ref2 = initialRef(waypoints_2);

% constant speed reference
ref1.uRefNominal = 1;
ref2.uRefNominal = 1;

%% Initialize Vehicles
% initial yaw value
yawInit = 90; 
vcorr = [0 0];

% establish structure
ASV1 = ASV_variables(sim, ref1.start, yawInit, 1);
ASV2 = ASV_variables(sim, ref2.start, yawInit, 2);

%% Simulation
for t = sim.time
    %% Update Reference, lawnmower
    if ASV1.gamma >= 1
        [ref1, ASV1] = componentPath(ASV1, waypoints_1, ref1);
    end
    
    if ASV2.gamma >= 1
        [ref2, ASV2] = componentPath(ASV2, waypoints_2, ref2);
    end
    
    %% Coordination
    if ASV1.section == ASV2.section
        [vcorr] = coordinationMaster(ASV1, ASV2);
    else
        vcorr = [ 0 0 ];
    end
    ref1.uRef = ref1.uRefNominal + vcorr(1);
    ref2.uRef = ref2.uRefNominal + vcorr(2);
    
    %% Calculate References
    [ref1.yawRef, ASV1] = pathFollowerASV(ASV1, ref1);
    [ref2.yawRef, ASV2] = pathFollowerASV(ASV2, ref2);

    %% Simulate Vehicles
    % ASV 1
    [ASV1] = innerLoopASV(ref1, ASV1);
    [ASV2] = innerLoopASV(ref2, ASV2);
    
    displayProgress(ASV1);

end

%% Plotting
plotTrajectory(ASV1,ASV2);
plotCoordination(ASV1,ASV2);
plotCrossTrackError(ASV1,ASV2);