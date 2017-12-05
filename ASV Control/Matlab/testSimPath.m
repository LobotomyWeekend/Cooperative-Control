%% Testing ASV Path Following
% The simulation can be varied using the Simulation Inputs, then the
% simulation is run by looping over time and using the path following
% controllers. 
% The file will also display plots of trajectory, internal
% command history, cross track error, and coordination states.

clear all;
close all;
clc;

complete = 1;
section = 0;

%% Simulation Inputs
% time
sim.Ts   = 0.01;
sim.Tend = 240;
sim.time = 0:sim.Ts:sim.Tend;

% waypoints
ref.pathType = 1;
ref.start = [0;0];
ref.finish = [0;10];

% constant speed reference
ref.uRef = 1;
ref.uRefNominal = 0.99;

% initial yaw value
yawInit = 90; 

%% Initialize Vehicles
% establish structure
ASV1 = ASV_variables(sim, ref.start, yawInit, 1);

%% Simulation
for t = sim.time    
    %% Calculate References
    [ref.yawRef, ASV1] = pathFollowerASV(ASV1, ref);      

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
plotTrajectory(ASV1);
plotCoordination(ASV1);
plotCrossTrackError(ASV1);

% figure('Name','Command Hist');
% hold on;
% grid on;
% subplot(2,1,1);
% plot(ASV1.time, ASV1.speedCommand_plot(1:length(ASV1.time)));
% subplot(2,1,2);
% plot(ASV1.time, ASV1.headingCommand_plot(1:length(ASV1.time)));
% hold off;