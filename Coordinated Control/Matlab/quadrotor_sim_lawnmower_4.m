%% Simulates UAV-UAV Cooperation
% Two UAVs working in coordination for any suitable path type, plots
% trajectory, cross track error, and coordination states over time.

close all
clear all
clc

global height
height = -1;

%% Simulation Inputs
% constants
vcorr = [0;0;0;0];
complete = 1;

% time
sim.Ts   = 0.01;
sim.Tend = 540;
sim.time = 0:sim.Ts:sim.Tend;

% waypoints
length_line = 25;
diameter_arc_min = 10;
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
% UAV1
UAV1 = quad_variables(sim, 1, ref1.start);
UAV1 = quad_dynamics_nonlinear(UAV1);
UAV1.ref = ref1;
% UAV2
UAV2 = quad_variables(sim, 2, ref2.start);
UAV2 = quad_dynamics_nonlinear(UAV2);
UAV2.ref = ref2;
% UAV3
UAV3 = quad_variables(sim, 3, ref3.start);
UAV3 = quad_dynamics_nonlinear(UAV3);
UAV3.ref = ref3;
% UAV3
UAV4 = quad_variables(sim, 4, ref4.start);
UAV4 = quad_dynamics_nonlinear(UAV4);
UAV4.ref = ref4;

vCorr = [0;0;0;0];

%% Run The Simulation Loop
i = 1;
for t = sim.time
    % Display Progression
    displayProgress(UAV1);
    
    %% Path Followers
    UAV1 = pathFollowerUAV(UAV1, UAV1.ref);
    UAV2 = pathFollowerUAV(UAV2, UAV2.ref);
    UAV3 = pathFollowerUAV(UAV3, UAV3.ref);
    UAV4 = pathFollowerUAV(UAV4, UAV4.ref);
    
    %% Lawnmower
    % Update Reference (Waypoint)
    if UAV1.gamma >= 1
        [ref1, UAV1] = componentPath(UAV1, ref1.waypoints, ref1);
        UAV1.ref = ref1;
    end
    
    if UAV2.gamma >= 1
        [ref2, UAV2] = componentPath(UAV2, ref2.waypoints, ref2);
        UAV2.ref = ref2;
    end
    
    if UAV3.gamma >= 1
        [ref3, UAV3] = componentPath(UAV3, ref3.waypoints, ref3);
        UAV3.ref = ref3;
    end
    
    if UAV4.gamma >= 1
        [ref4, UAV4] = componentPath(UAV4, ref4.waypoints, ref4);
        UAV4.ref = ref4;
    end
    
    %% Coordination
    % Master coordination controller
    if UAV1.section == UAV2.section && UAV2.section == UAV3.section && UAV3.section == UAV4.section
        %get correction velocity
        vcorr = coordinationMaster(UAV1, UAV2, UAV3, UAV4);
    else
        vcorr = [ 0 0 0 0];
    end
    % update speed references
    UAV1.ref.uRef = UAV1.ref.uRefNominal + vcorr(1); %UAV
    UAV2.ref.uRef = UAV2.ref.uRefNominal + vcorr(2); %UAV
    UAV3.ref.uRef = UAV3.ref.uRefNominal + vcorr(3); %UAV
    UAV4.ref.uRef = UAV4.ref.uRefNominal + vcorr(4); %UAV
        
    %% Inner Loop Dynamics
    UAV1 = innerLoopUAV(UAV1);
    UAV2 = innerLoopUAV(UAV2);
    UAV3 = innerLoopUAV(UAV3);
    UAV4 = innerLoopUAV(UAV4);
    
    vCorr_hist(:,i) = vcorr';
    i = i + 1;
end

%% Plots
% trajectory
plotTrajectory(UAV1, UAV2, UAV3, UAV4);
% coordination state
plotCoordination(UAV1, UAV2, UAV4);
% cross track error
plotCrossTrackError(UAV1, UAV2, UAV3, UAV4);
% plot vcorr hist
plotVcorr(vCorr_hist, sim.time);
