%% Coordination between 1 UAV + 1 ASV
% Test simulation for cooperative motion between the two types of vehicle,
% for any supported path type.
%% Initialize Workspace
clear all;
close all;
clc;

global height
height = -1;

% constants
complete = 1.0;

%% Simulation Time
sim.Ts = 0.01;
sim.Tend = 650;
sim.time = 0:sim.Ts:sim.Tend;

%% Waypoints + Refs
% waypoints
length_line = 20;
diameter_arc_min = 25;
offset = 5;
segments = 5;
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
ref1.uRefNominal = 0.25;
ref1.uRef = ref1.uRefNominal;

ref2.uRefNominal = 0.25;
ref2.uRef = ref2.uRefNominal;

%% UAV Setup
% initialize vehicle structure
UAV1 = quad_variables(sim, 1, ref1.start);
UAV1 = quad_dynamics_nonlinear(UAV1);
UAV1.ref = ref1;

%% ASV Setup
% initial values
yawInit = 90;
% initialize vehicle structure
ASV2 = ASV_variables(sim, ref2.start, yawInit, 2);
ASV2.ref = ref2;

vCorr = [0,0];

i = 1;
%% SIMULATION
for t =  sim.time
    % Display Progression
    displayProgress(UAV1);
    
    %% Path Followers
    UAV1 = pathFollowerUAV(UAV1, UAV1.ref);
    [ref2.yawRef, ASV2] = pathFollowerASV(ASV2, ref2);
    
    %% Lawnmower
    % Update references along component path
    if UAV1.gamma >= 1
        [ref1, UAV1] = componentPath(UAV1, waypoints_1, ref1);
        UAV1.ref = ref1;
    end
    
    if ASV2.gamma >= 1
        [ref2, ASV2] = componentPath(ASV2, waypoints_2, ref2);
    end
    
    %% Coordination
    if UAV1.section == ASV2.section && UAV1.counter > 100
        vCorr = coordinationMaster(UAV1, ASV2);
    else
        vCorr = [ 0 0 ];
    end
    
    % Update Speed Reference
    UAV1.ref.uRef = UAV1.ref.uRefNominal + vCorr(1); %UAV
    ref2.uRef = ref2.uRefNominal + vCorr(2); %ASV
    if ref2.uRef < 0
        ref2.uRef = 0;
    end

    %% Simulate Vehicles
    UAV1 = innerLoopUAV(UAV1);
    ASV2 = innerLoopASV(ref2, ASV2);
    
    vCorr_hist(i,:) = vCorr;
    i = i + 1; 
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

%% TEST PLOTS
figure('Name','Correction velocity');
hold on
grid on
plot(sim.time, vCorr_hist(:,1));
plot(sim.time, vCorr_hist(:,2));
legend('UAV','ASV');
xlabel('Time (s)');
ylabel('V_{corr} (m s^{-1})');
title('ASV-UAV Cooperation Correction Velocities');
hold off

clc