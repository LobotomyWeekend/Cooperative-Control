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
sim.Tend = 120;

%% Path Variables & References
ref.pathType = 1;
ref.start = [0; 0]; % also vehicle's initial position
ref.finish = [20; 20];
ref.uRefNominal = 0;
ref.uRef = 0.5;
ref.yawRef = 0;

%% Initialize Vehicle
UAV = quad_variables(sim,ref.start);
UAV = quad_dynamics_nonlinear(UAV);

%% Run The Simulation Loop
i = 1;
for t = UAV.t_plot
    
    if ~mod(t,20)
        ref.yawRef = ref.yawRef + 45/2;
    end
    
    % Generate speed reference
    UAV = parameterizeSpeed(UAV, ref);
    
    % Inner Loop Dynamics and Controllers
    UAV = innerLoopUAV(UAV);
    
    % Save reference Values
    uRef_plot(:,i) = [UAV.X_dot_GF_des; UAV.Y_dot_GF_des];
    yawRef_plot(i) = ref.yawRef;
    heading = atand(UAV.Y_dot / UAV.X_dot);
    if heading < 0
        heading = heading + 180;
    end
    heading_plot(i) = heading;
    
    i = i + 1;
end

%% Plots
% trajectory
plotTrajectory(UAV);

figure('Name', 'Speed Plot');
subplot(2,1,1)
xlabel('Time (s)');
ylabel('X Speed (m/s)');
hold on
grid on
plot(UAV.t_plot, uRef_plot(1,:), '--');
plot(UAV.t_plot, UAV.X_dot_plot(1:length(UAV.t_plot)));
hold off

subplot(2,1,2)
xlabel('Time (s)');
ylabel('Y Speed (m/s)');
hold on
grid on
plot(UAV.t_plot, uRef_plot(2,:), '--');
plot(UAV.t_plot, UAV.Y_dot_plot(1:length(UAV.t_plot)));
hold off

figure('Name', 'Heading Plot');
xlabel('Time(s)');
ylabel('Heading (deg)');
hold on
grid on
plot(UAV.t_plot, yawRef_plot, '--');
plot(UAV.t_plot, heading_plot);
hold off

% % coordination state
% plotCoordination(UAV);
% % cross track error
% plotCrossTrackError(UAV);
