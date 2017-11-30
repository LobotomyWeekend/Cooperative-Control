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
sim.Tend = 360;
sim.time = 0:sim.Ts:sim.Tend;

%% UAV Setup
% Vehicle 1
ref1.pathType = 1;
ref1.start = [0; 0];
ref1.finish = [20; 20];
% constant speed reference
ref1.uRefNominal = 1;
% initialize vehicle structure
UAV1 = quad_variables(sim, 1, ref1.start);
UAV1 = quad_dynamics_nonlinear(UAV1);

%% ASV Setup
% waypoints
ref2.pathType = 1;
ref2.start = [5;0];
ref2.finish = [25;20];
% constant speed reference
ref2.uRefNominal = 1;
% initial values
yawInit = 90;
% initialize vehicle structure
ASV2 = ASV_variables(sim, ref2.start, yawInit, 2);

%% Constants (SIMPLIFIED SIMULATION)
vCorr = [0;0];
ref2.uRef = ref2.uRefNominal;


%% SIMULATION
for t =  sim.time
    %% Coordination
    vCorr = coordinationMaster(UAV1, ASV2);
    ref2.uRef = ref2.uRefNominal + vCorr(ASV2.vehicleID);

    %% Path Following
    UAV1 = lookaheadPathFollowerUAV(UAV1, ref1, vCorr(UAV1.vehicleID));
    [ref2.yawRef, ASV2] = pathFollowerASV(ASV2, ref2);
    
    %% End Condition
    UAV1 = endConditionUAV(UAV1, ref1, complete);
    [ref2, ASV2] = endConditionASV(ASV2, ref2, complete);

    %% Simulate Vehicles
    UAV1 = innerLoopUAV(UAV1);
    ASV2 = innerLoopASV(ref2, ASV2);

    UAV1.init = 1;
    
    %% Display Progression
    clc
    progress = floor(ASV2.counter / length(ASV2.time) * 100);
    display = [num2str(progress), '% progression'];
    disp(display);
end

%% Plotting
figure('Name','Trajectory');
hold on;
grid on;
plot(UAV1.X_plot, UAV1.Y_plot);
plot(ASV2.X_plot, ASV2.Y_plot);
legend('UAV','ASV');
axis('equal');
hold off;

figure('Name','Coordination States');
hold on;
grid on;
plot(sim.time, UAV1.gamma_plot(1:length(sim.time)));
plot(sim.time, ASV2.gamma_plot(1:length(sim.time)));
legend('UAV1','ASV2');
hold off;