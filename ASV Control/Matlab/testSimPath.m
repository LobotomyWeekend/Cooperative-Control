%% TESTING ASV INNER LOOP
% function which mimics the behaviour of the simulink model
% i.e. can be placed inside a time loop with ref values changed
% dynamically.
clear all;
close all;
clc;

complete = 1;

%% Simulation Inputs
% time
sim.Ts   = 0.01;
sim.Tend = 240;
sim.time = 0:sim.Ts:sim.Tend;

% waypoints
ref.pathType = 2;
ref.start = [0;0];
ref.finish = [10;0];

% constant speed reference
ref.uRef = 1;
ref.uRefNominal = 0.99;

%% Initialize Vehicles
% initial yaw value
yawInit = 0; 
% establish structure
ASV1 = ASV_variables(sim, ref.start, yawInit, 1);

%% Simulation
for t = sim.time    
    %% Calculate References
    [ref.yawRef, ASV1] = pathFollowerASV(ASV1, ref);
    
    %% End Condition
    [ref, ASV1] = endConditionASV(ASV1, ref, complete);

    %% Simulate Vehicles
    % ASV 1
    [ASV1] = innerLoopASV(ref, ASV1);
    
    %% Display Progression
    clc
    progress = floor(ASV1.counter / length(ASV1.time) * 100);
    display = [num2str(progress), '% progression'];
    disp(display);

end

%% Plotting
figure('Name','Trajectory');
hold on;
grid on;
plot(ASV1.X_plot, ASV1.Y_plot);
hold off;

figure('Name','Command Hist');
hold on;
grid on;
subplot(2,1,1);
plot(ASV1.time, ASV1.speedCommand_plot(1:length(ASV1.time)));
subplot(2,1,2);
plot(ASV1.time, ASV1.headingCommand_plot(1:length(ASV1.time)));
hold off;

figure('Name','Cross Track Error');
hold on;
grid on;
plot(ASV1.time, ASV1.error_crossTrack_plot(1:length(ASV1.time)));
hold off

figure('Name','Coordination States');
hold on;
grid on;
plot(sim.time, ASV1.gamma_plot(1:length(sim.time)));
hold off;

