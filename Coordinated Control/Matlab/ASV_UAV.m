%% Coordination between 1 UAV + 1 ASV
% Test simulation for cooperative motion between the two types of vehicle,
% for any supported path type.
%% Initialize Workspace
clear all;
close all;
clc;

% constants
complete = 1.0;

%% Simulation inputs
sim.Ts = 0.01;
sim.Tend = 120;
sim.time = 0:sim.Ts:sim.Tend;

%% UAV Setup
% Vehicle 1
ref1.pathType = 2;
ref1.start = [2; 0];
ref1.finish = [18; 0];
ref1.waypoints = ref_waypoints(ref1);
% constant speed reference
ref1.uRefNominal = 0.25;
% initialize vehicle structure
UAV1 = quad_variables(sim, 1, ref1.start);
UAV1 = quad_dynamics_nonlinear(UAV1);
UAV1.ref = ref1;

%% ASV Setup
% waypoints
ref2.pathType = 2;
ref2.start = [0; 0];
ref2.finish = [20; 0];
ref2.waypoints = ref_waypoints(ref2);
% constant speed reference
ref2.uRefNominal = 0.5;
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
    
    % Coordination
    if UAV1.counter > 100
        vCorr = coordinationMaster(UAV1, ASV2);
    end
    
    % Update Speed Reference
    UAV1.ref.uRef = ref1.uRefNominal + vCorr(1);
    ref2.uRef = ref2.uRefNominal + vCorr(2);

    % Path Following
    UAV1 = pathFollowerUAV(UAV1, ref1);
    [ref2.yawRef, ASV2] = pathFollowerASV(ASV2, ref2);

    % Simulate Vehicles
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
hold off

clc