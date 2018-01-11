%% COORDINATION ASV-ASV
% simulation of two ASV vehicles working in cooperation
clear all;
close all;
clc;

%% Simulation Time
sim.Ts = 0.01;
sim.Tend = 600;
sim.time = 0:sim.Ts:sim.Tend;

%% Waypoints + Refs
% waypoints
length_line = 20;
diameter_arc_min = 25;
offset = 2;
segments = 5;
no_vehicles = 4;

% lawnmower path for 3 vehicles
waypoints = waypointsMultiLawnmower(length_line, diameter_arc_min, offset, segments, no_vehicles);

% each vehicle's waypoints
waypoints_1 = waypoints(1:3,:);
waypoints_2 = waypoints(4:6,:);
waypoints_3 = waypoints(7:9,:);
waypoints_4 = waypoints(10:12,:);

% constand speed reference
nominal_velocity = 0.25; % m/s

% initial reference
ref1 = initialRef(waypoints_1, nominal_velocity);
ref2 = initialRef(waypoints_2, nominal_velocity);
ref3 = initialRef(waypoints_3, nominal_velocity);
ref4 = initialRef(waypoints_4, nominal_velocity);

%% Initialize Vehicles
% initial yaw value
yawInit = 90; 
vcorr = [0 0 0 0];

% establish structure
ASV1 = ASV_variables(sim, ref1.start, yawInit, 1);
ASV2 = ASV_variables(sim, ref2.start, yawInit, 2);
% UAV3
UAV3 = quad_variables(sim, 3, ref3.start);
UAV3 = quad_dynamics_nonlinear(UAV3);
% UAV3
UAV4 = quad_variables(sim, 4, ref4.start);
UAV4 = quad_dynamics_nonlinear(UAV4);

% combine ref and vehicle
ASV1.ref = ref1;
ASV2.ref = ref2;
UAV3.ref = ref3;
UAV4.ref = ref4;

%% Simulation
i = 1;
for t = sim.time
    displayProgress(ASV1);
    %% Path Followers
    [ref1.yawRef, ASV1] = pathFollowerASV(ASV1, ref1);
    [ref2.yawRef, ASV2] = pathFollowerASV(ASV2, ref2);
    UAV3 = pathFollowerUAV(UAV3, UAV3.ref);
    UAV4 = pathFollowerUAV(UAV4, UAV4.ref);
    %% Lawnmower
    % Update reference at end of each path segment
    if ASV1.gamma >= 1
        [ref1, ASV1] = componentPath(ASV1, waypoints_1, ref1);
    end
    if ASV2.gamma >= 1
        [ref2, ASV2] = componentPath(ASV2, waypoints_2, ref2);
    end
    if UAV3.gamma >= 1
        [ref3, UAV3] = componentPath(UAV3, waypoints_3, ref3);
        UAV3.ref = ref3;
    end
    if UAV4.gamma >= 1
        [ref4, UAV4] = componentPath(UAV4, waypoints_4, ref4);
        UAV4.ref = ref4;
    end
    %% Coordination
    if ASV1.section == ASV2.section && ASV2.section == UAV3.section && UAV3.section == UAV4.section
        vCorr = coordinationMaster(ASV1, ASV2, UAV3, UAV4);
    else
        vCorr = [ 0 0 0 0 ];
    end
    %% Update Speed Reference
    ref1.uRef = ref1.uRefNominal + vCorr(1); %ASV1
    ref2.uRef = ref2.uRefNominal + vCorr(2); %ASV2
    UAV3.ref.uRef = UAV3.ref.uRefNominal + vCorr(3); %UAV3
    UAV4.ref.uRef = UAV4.ref.uRefNominal + vCorr(4); %UAV4
    %% Simulate Vehicles
    ASV1 = innerLoopASV(ref1, ASV1);
    ASV2 = innerLoopASV(ref2, ASV2);
    UAV3 = innerLoopUAV(UAV3);
    UAV4 = innerLoopUAV(UAV4);
    
    vCorr_hist(i,:) = vCorr;
    i = i + 1; 
end
disp('Finishing Up...');

%% Plotting
plotTrajectory(ASV1,ASV2,UAV3,UAV4);
plotCoordination(ASV1,ASV2,UAV3,UAV4);
plotCrossTrackError(ASV1,ASV2,UAV3,UAV4);
plotVcorr(vCorr_hist, sim.time);

clc