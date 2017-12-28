%% Simulates quadrotor dynamics and implements a control algorithm
% This simulation is a new version 26/12/2017 which controls the UAV in
% absolute velocity and heading. This is the same method as with the ASV,
% and has a number of advantages over waypoint control.

clear all;
close all;
clc;

%% Initialize Workspace
% constants
complete = 1.0;
vCorr = 0.0;

%% Simulation inputs
sim.Ts = 0.01;
sim.Tend = 70;

%% Path Variables & References
ref.pathType = 2;
ref.start = [0; 0]; % also vehicle's initial position
ref.finish = [20; 0];
ref.uRefNominal = 0.5;
ref.uRef = 0.5;
ref.yawRef = 210;

%% Initialize Vehicle
UAV = quad_variables(sim,ref.start);
UAV = quad_dynamics_nonlinear(UAV);
UAV.ref = ref;

%% Run The Simulation Loop
i = 1;
for t = UAV.t_plot
    % Display Progression
    displayProgress(UAV);
    
    % Path Follower
    [UAV, yawRefTEST] = pathFollowerUAV(UAV, ref);

    % Inner Loop Dynamics and Controllers
    UAV = innerLoopUAV(UAV);
    
    UAV = endConditionUAV(UAV);
    
    % TEST THE CONTROLLERS %
%     uRef_plot(:,i) = [UAV.X_dot_GF_des; UAV.Y_dot_GF_des];
%     yawRef_plot(i) = yawRefTEST;
%     heading = atan2d(UAV.Y_dot, UAV.X_dot);
%     heading_plot(i) = heading;
    
    i = i + 1;
end
clc
disp('Finishing Up...');

%% Plots
% trajectory
plotTrajectory(UAV);
% coordination state
% plotCoordination(UAV);
% cross track error
plotCrossTrackError(UAV);

%% Analysis of controllers
% figure('Name', 'Speed Plot');
% subplot(2,1,1)
% xlabel('Time (s)');
% ylabel('X Speed (m/s)');
% hold on
% grid on
% plot(UAV.t_plot, uRef_plot(1,:), '--');
% plot(UAV.t_plot, UAV.X_dot_plot(1:length(UAV.t_plot)));
% hold off
% 
% subplot(2,1,2)
% xlabel('Time (s)');
% ylabel('Y Speed (m/s)');
% hold on
% grid on
% plot(UAV.t_plot, uRef_plot(2,:), '--');
% plot(UAV.t_plot, UAV.Y_dot_plot(1:length(UAV.t_plot)));
% hold off
% 
% figure('Name', 'Heading Plot');
% xlabel('Time(s)');
% ylabel('Heading (deg)');
% hold on
% grid on
% plot(UAV.t_plot, yawRef_plot, '--');
% plot(UAV.t_plot, heading_plot);
% hold off

clc;