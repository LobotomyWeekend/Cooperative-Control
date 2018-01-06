%% COORDINATION ASV-ASV
% simulation of two ASV vehicles working in cooperation
clear all;
close all;
clc;

%% Simulation Inputs
% time
sim.Ts   = 0.1;
sim.Tend = 60;
sim.time = 0:sim.Ts:sim.Tend;

% waypoints
length_line = 15;
diameter_arc_min = 25;
offset = 5;
segments = 5;
no_vehicles = 4;

% lawnmower path for 3 vehicles
waypoints = waypointsMultiLawnmower(length_line, diameter_arc_min, offset, segments, no_vehicles);

% each vehicle's waypoints
waypoints_1 = waypoints(1:3,:);
waypoints_2 = waypoints(4:6,:);
waypoints_3 = waypoints(7:9,:);
waypoints_4 = waypoints(10:12,:);

% initial reference
ref1 = initialRef(waypoints_1);
ref2 = initialRef(waypoints_2);
ref3 = initialRef(waypoints_3);
ref4 = initialRef(waypoints_4);

% constant speed reference
ref1.uRefNominal = 0.5;
ref2.uRefNominal = 0.5;
ref3.uRefNominal = 0.5;
ref4.uRefNominal = 0.5;

ref1.uRef = ref1.uRefNominal;
ref2.uRef = ref2.uRefNominal;
ref3.uRef = ref3.uRefNominal;
ref4.uRef = ref4.uRefNominal;


%% Initialize Vehicles
% initial yaw value
yawInit = 90; 
vcorr = [0; 0; 0; 0];

% establish structure
ASV1 = ASV_variables(sim, ref1.start, yawInit, 1);
ASV2 = ASV_variables(sim, ref2.start, yawInit, 2);
ASV3 = ASV_variables(sim, ref3.start, yawInit, 3);
ASV4 = ASV_variables(sim, ref4.start, yawInit, 4);

ASV1.ref = ref1;
ASV2.ref = ref2;
ASV3.ref = ref3;
ASV4.ref = ref4;

%% Simulation
i = 1;
for t = sim.time
    displayProgress(ASV1);
    %% Path Follower
    [ref1.yawRef, ASV1] = pathFollowerASV(ASV1, ref1);
    [ref2.yawRef, ASV2] = pathFollowerASV(ASV2, ref2);
    [ref3.yawRef, ASV3] = pathFollowerASV(ASV3, ref3);
    [ref4.yawRef, ASV4] = pathFollowerASV(ASV4, ref4);    

    %% Update Reference, lawnmower
    if ASV1.gamma >= 1
        [ref1, ASV1] = componentPath(ASV1, waypoints_1, ref1);
    end
    if ASV2.gamma >= 1
        [ref2, ASV2] = componentPath(ASV2, waypoints_2, ref2);
    end
    if ASV3.gamma >= 1
        [ref2, ASV3] = componentPath(ASV3, waypoints_3, ref3);
    end
    if ASV4.gamma >= 1
        [ref4, ASV4] = componentPath(ASV4, waypoints_4, ref4);
    end
    
    %% Coordination
    if ASV1.section == ASV2.section && ASV2.section == ASV3.section && ASV3.section == ASV4.section
        [vcorr] = coordinationMaster(ASV1, ASV2, ASV3, ASV4);
    else
        vcorr = [0 0 0 0];
    end
    ref1.uRef = ref1.uRefNominal + vcorr(1);
    ref2.uRef = ref2.uRefNominal + vcorr(2);
    ref3.uRef = ref3.uRefNominal + vcorr(3);
    ref4.uRef = ref4.uRefNominal + vcorr(4);
    
   
    %% Simulate Vehicles
    % ASV 1
    [ASV1] = innerLoopASV(ref1, ASV1);
    [ASV2] = innerLoopASV(ref2, ASV2);
    [ASV3] = innerLoopASV(ref3, ASV3);
    [ASV4] = innerLoopASV(ref4, ASV4);
    
    %% Save Data
    vCorr_hist(:,i) = vcorr';
    i = i + 1;    
end
disp('Finishing Up...');
%% Plotting
plotTrajectory(ASV1,ASV2, ASV3, ASV4);
plotCoordination(ASV1,ASV2,ASV3,ASV4);
plotCrossTrackError(ASV1,ASV2,ASV3,ASV4);
plotVcorr(vCorr_hist, sim.time);
clc