%% TESTING ASV INNER LOOP
% function which mimics the behaviour of the simulink model
% i.e. can be placed inside a time loop with ref values changed
% dynamically.
clear all;
close all;

%% Simulation Inputs
% time
sim.Ts   = 0.1;
sim.Tend = 360;
sim.time = 0:sim.Ts:sim.Tend;

% waypoints
ref.start = [0;0];
ref.finish = [10;10];

%% Initialize Vehicles
% initial yaw value
yawInit = 0; 
% establish structure
ASV1 = ASV_variables(sim, ref.start, yawInit, 1);


%% Simulation
for t = sim.time
    %% Calculate References
    ref.yawRef = 120;
    ref.uRef = 1;

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

