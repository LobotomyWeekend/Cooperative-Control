%% Simulates UAV-UAV Cooperation
% Two UAVs working in coordination for any suitable path type, plots
% trajectory, cross track error, and coordination states over time.

close all
clear all
clc

%% Simulation Inputs
% constants
vCorr = [0, 0];
complete = 1;

% time
sim.Ts   = 0.01;
sim.Tend = 240;
sim.time = 0:sim.Ts:sim.Tend;

% waypoints
length_line = 25;
diameter_arc_min = 10;
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
ref1.uRefNominal = 0.5;
ref1.uRef = ref1.uRefNominal;
ref2.uRefNominal = 0.5;
ref2.uRef = ref2.uRefNominal;

%% Initialize Vehicles
% UAV1
UAV1 = quad_variables(sim, 1, ref1.start);
UAV1 = quad_dynamics_nonlinear(UAV1);
UAV1.ref = ref1;
% UAV2
UAV2 = quad_variables(sim, 2, ref2.start);
UAV2 = quad_dynamics_nonlinear(UAV2);
UAV2.ref = ref2;

%% Run The Simulation Loop
i = 1;
for t = sim.time
    % Update Reference (Waypoint)
    if UAV1.gamma >= 1
        [ref1, UAV1] = componentPath(UAV1, waypoints_1, ref1);
        UAV1.ref = ref1;
    end
    
    if UAV2.gamma >= 1
        [ref2, UAV2] = componentPath(UAV2, waypoints_2, ref2);
        UAV2.ref = ref2;
    end
    
    % Master coordination controller
    if UAV1.section == UAV2.section
        vCorr = coordinationMaster(UAV1, UAV2);
    else 
        vCorr = [ 0 0 ];
    end
    UAV1.ref.uRef = UAV1.ref.uRefNominal + vCorr(1);
    UAV2.ref.uRef = UAV2.ref.uRefNominal + vCorr(2);
    
    % Path Follower
    UAV1 = pathFollowerUAV(UAV1, UAV1.ref);
    UAV2 = pathFollowerUAV(UAV2, UAV2.ref);
    
    % Inner Loop Dynamics and Controllers
    UAV1 = innerLoopUAV(UAV1);
    UAV2 = innerLoopUAV(UAV2);
    
    displayProgress(UAV1);
end

%% Plots
% trajectory
plotTrajectory(UAV1, UAV2);
% coordination state
plotCoordination(UAV1, UAV2);
% % cross track error
% plotCrossTrackError(UAV1, UAV2);
