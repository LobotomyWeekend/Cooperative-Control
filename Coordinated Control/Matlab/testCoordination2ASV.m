%% TESTING ASV INNER LOOP
% function which mimics the behaviour of the simulink model
% i.e. can be placed inside a time loop with ref values changed
% dynamically.
clear all;
close all;
clc;

%% Simulation Inputs
% time
sim.Ts   = 0.01;
sim.Tend = 120;
sim.time = 0:sim.Ts:sim.Tend;

% waypoints
ref1.pathType = 1;
ref1.start = [0;0];
ref1.finish = [10;10];

ref2.pathType = 1;
ref2.start = [5;0];
ref2.finish = [15;10];

% constant speed reference
ref1.uRefNominal = 1;
ref2.uRefNominal = 1;

%% Initialize Vehicles
% initial yaw value
yawInit = 0; 

% establish structure
ASV1 = ASV_variables(sim, ref1.start, yawInit, 1);
ASV2 = ASV_variables(sim, ref2.start, yawInit, 2);

%% Simulation
for t = sim.time
    %% Coordination
    [vcorr] = coordinationMaster(ASV1, ASV2);
    ref1.uRef = ref1.uRefNominal + vcorr(1);
    ref2.uRef = ref2.uRefNominal + vcorr(2);
    
    %% Calculate References
    [ref1.yawRef, ASV1] = pathFollowerASV(ASV1, ref1);
    [ref2.yawRef, ASV2] = pathFollowerASV(ASV2, ref2);

    %% Simulate Vehicles
    % ASV 1
    [ASV1] = innerLoopASV(ref1, ASV1);
    [ASV2] = innerLoopASV(ref2, ASV2);

end

%% Plotting
figure('Name','Trajectory');
hold on;
grid on;
plot(ASV1.X_plot, ASV1.Y_plot);
plot(ASV2.X_plot, ASV2.Y_plot);
xlabel('x (m)');
ylabel('y (m)');
hold off;

figure('Name','Coordination States');
hold on;
grid on;
plot(sim.time, ASV1.gamma_plot);
plot(sim.time, ASV2.gamma_plot);
legend('ASV1','ASV2');
hold off;