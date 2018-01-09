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
length_line = 10;
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

% initial reference
ref1 = initialRef(waypoints_1);
ref2 = initialRef(waypoints_2);
ref3 = initialRef(waypoints_3);
ref4 = initialRef(waypoints_4);

% constant speed reference
ref1.uRefNominal = 0.25;
ref1.uRef = ref1.uRefNominal;
ref2.uRefNominal = 0.25;
ref2.uRef = ref2.uRefNominal;
ref3.uRefNominal = 0.25;
ref3.uRef = ref3.uRefNominal;
ref4.uRefNominal = 0.25;
ref4.uRef = ref4.uRefNominal;

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
    
    %% Update Reference, lawnmower
    if ASV1.gamma >= 1
        [ref1, ASV1] = componentPath(ASV1, waypoints_1, ref1);
    end
    
    if ASV2.gamma >= 1
        [ref2, ASV2] = componentPath(ASV2, waypoints_2, ref2);
    end
    
    if UAV3.gamma >= 1
        [ref3, UAV3] = componentPath(UAV3, ref3.waypoints, ref3);
        UAV3.ref = ref3;
    end
    
    if UAV4.gamma >= 1
        [ref4, UAV4] = componentPath(UAV4, ref4.waypoints, ref4);
        UAV4.ref = ref4;
    end
    
%     %% Coordination
%     if ASV1.section == ASV2.section && ASV2.section == UAV3.section && UAV3.section == UAV4.section
%         [vcorr] = coordinationMaster(ASV1, ASV2, UAV3, UAV4);
%     else
%         vcorr = [ 0 0 0 0];
%     end
%     ref1.uRef = ref1.uRefNominal + vcorr(1); %ASV
%     ref2.uRef = ref2.uRefNominal + vcorr(2); %ASV
%     UAV3.ref.uRef = UAV3.ref.uRefNominal + vcorr(3); %UAV
%     UAV4.ref.uRef = UAV4.ref.uRefNominal + vcorr(4); %UAV

    %% Path Following
    [ref1.yawRef, ASV1] = pathFollowerASV(ASV1, ref1);
    [ref2.yawRef, ASV2] = pathFollowerASV(ASV2, ref2);
    UAV3 = pathFollowerUAV(UAV3, UAV3.ref);
    UAV4 = pathFollowerUAV(UAV4, UAV4.ref);
    %% Simulate Vehicles
    % ASV 1
    ASV1 = innerLoopASV(ref1, ASV1);
    ASV2 = innerLoopASV(ref2, ASV2);
    UAV3 = innerLoopUAV(UAV3);
    UAV4 = innerLoopUAV(UAV4);
    
    
    vCorr_hist(:,i) = vcorr';
    i = i + 1;
end

%% Plotting
plotTrajectory(ASV1,ASV2,UAV3,UAV4);
plotCoordination(ASV1,ASV2,UAV3,UAV4);
plotCrossTrackError(ASV1,ASV2,UAV3,UAV4);
plotVcorr(vCorr_hist, UAV3.t_plot);