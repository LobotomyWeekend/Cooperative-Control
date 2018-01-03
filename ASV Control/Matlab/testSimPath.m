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
sim.Tend = 360;
sim.time = 0:sim.Ts:sim.Tend;

% waypoints
ref.pathType = 3;
ref.start = [0;0];
ref.finish = [20;0];

% constant speed reference
ref.uRef = 0.5;
ref.uRefNominal = 0.5;

% initial yaw value
yawInit = 270; 

%% Initialize Vehicles
% establish structure
ASV1 = ASV_variables(sim, ref.start, yawInit, 1);
ASV1.ref = ref;

i = 1;
%% Simulation
for t = sim.time   
    displayProgress(ASV1);
    
    %% Calculate References
    [ref.yawRef, ASV1] = pathFollowerASV(ASV1, ref);      

    %% Simulate Vehicles
    % ASV 1
    [ASV1] = innerLoopASV(ref, ASV1);
    
    yaw_hist(i) = ASV1.Yaw;
    yaw_ref_hist(i) = ref.yawRef;
    i = i + 1;
    
end

%% Plotting
plotTrajectory(ASV1);
plotCoordination(ASV1);
plotCrossTrackError(ASV1);

figure('Name','Command Hist');
hold on;
grid on;
subplot(2,1,1);
plot(ASV1.time, ASV1.speedCommand_plot(1:length(ASV1.time)));
subplot(2,1,2);
plot(ASV1.time, ASV1.headingCommand_plot(1:length(ASV1.time)));
hold off;

figure('Name','Yaw Hist');
hold on
grid on
plot(ASV1.time, yaw_hist);
plot(ASV1.time, yaw_ref_hist, '--');
hold off