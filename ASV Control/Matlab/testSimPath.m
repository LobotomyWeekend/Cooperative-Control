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
sim.Tend = 240;
sim.time = 0:sim.Ts:sim.Tend;

% waypoints
ref.pathType = 2;
ref.start = [0;0];
ref.finish = [10;0];

% constant speed reference
ref.uRef = 1;

%% Initialize Vehicles
% initial yaw value
yawInit = 0; 
% establish structure
ASV1 = ASV_variables(sim, ref.start, yawInit, 1);

%% Simulation
for t = sim.time    
    %% Calculate References
    [ref.yawRef, ASV1] = pathFollowerASV(ASV1, ref);

    %% Simulate Vehicles
    % ASV 1
    [ASV1] = innerLoopASV(ref, ASV1);

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
plot(ASV1.time, ASV1.speedCommand_plot(1:length(ASV1.time)));
plot(ASV1.time, ASV1.headingCommand_plot(1:length(ASV1.time)));
legend('Speed Command','Heading Command');
hold off;

figure('Name','Cross Track Error');
hold on;
grid on;
plot(ASV1.time, ASV1.error_crossTrack_plot(1:length(ASV1.time)));
hold off

