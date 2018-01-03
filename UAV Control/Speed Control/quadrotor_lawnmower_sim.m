%% Simulates quadrotor dynamics and implements a control algorithm

%% Initialize Workspace
clear all;
close all;
clc;

% constants
vCorr = 0.0;

%% Simulation inputs
sim.Ts = 0.01;
sim.Tend = 160;

%% Path Variables & References
% waypoints
length_line = 20;
diameter_arc = 20;
segments = 5;

[wayPoints, ref] = waypointsLawnmower(length_line, diameter_arc, segments);

% nominal speed
ref.uRefNominal = 0.25;
ref.uRef = ref.uRefNominal;

%% Initialize Vehicle
UAV = quad_variables(sim,ref.start);
UAV = quad_dynamics_nonlinear(UAV);
UAV.ref = ref;

%% Run The Simulation Loop
i = 1;
for t = UAV.t_plot
    % Display Progression
    displayProgress(UAV);
    
    % Lawnmower 
    if UAV.gamma >= 1
        [ref, UAV] = componentPath(UAV, wayPoints, ref);
        UAV.ref = ref;
    end
      
    % Path Follower
    UAV = pathFollowerUAV(UAV, UAV.ref);
    
    % Inner Loop Dynamics and Controllers
    UAV = innerLoopUAV(UAV);
    
    %% TESTS
    refHist(i) = ref;
    
    i=i+1;
    
end
clc
disp('Finishing Up...');
%% Plots
% trajectory
plotTrajectory(UAV);
% coordination state
plotCoordination(UAV);
% cross track error
plotCrossTrackError(UAV);
clc

%% TEST PLOTS
figure('Name', 'PathType')
plot(UAV.time(1:length(UAV.time)-1), [refHist.pathType]);