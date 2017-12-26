%% Simulates quadrotor on Lawnmower Path
% The path's waypoints are defined by the function waypointsLawnmower,
% which accepts arguments; swath length, arc diameter, and number of path
% segments. The vehicle is then simulated and commanded to follow this
% path. Once the simulation has completed (reached Tend) it will plot
% trajectory, coordination state, and cross track error.

%% Initialize Workspace
clear all;
close all;
clc;

% constants
complete = 1.0;
vCorr = 0.0;

%% Simulation inputs
sim.Ts = 0.01;
sim.Tend = 1440;

%% Path Variables & References
% waypoints
length_line = 50;
diameter_arc = 20;
segments = 20;

[wayPoints, ref] = waypointsLawnmower(length_line, diameter_arc, segments);

% nominal speed
ref.uRefNominal = 0.5;

%% Initialize Vehicle
UAV = quad_variables(sim,ref.start);
UAV = quad_dynamics_nonlinear(UAV);

%% Run The Simulation Loop
for t = UAV.t_plot
    % Lawnmower 
    if UAV.gamma >= 1
        [ref, UAV] = componentPath(UAV, wayPoints, ref);
    end
    
    % Coordination
    vCorr = coordinationMaster(UAV);
      
    % Path Follower
    UAV = lookaheadPathFollowerUAV(UAV, ref, vCorr);
    
    % Inner Loop Dynamics and Controllers
    UAV = innerLoopUAV(UAV);
    
    %% Display Progression
    clc
    progress = floor(UAV.counter / length(UAV.t_plot) * 100);
    display = [num2str(progress), '% progression'];
    disp(display);
end

%% Plots
% trajectory
plotTrajectory(UAV);
% coordination state
plotCoordination(UAV);
% cross track error
plotCrossTrackError(UAV);
