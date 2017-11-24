
%% Simulates quadrotor dynamics and implements a control algorithm
% Add Paths
addpath utilities

%% Initialize Workspace
clear all;
close all;
clc;

global Quad;
global Ref;
global sim;

% constants
complete = 1.0;

%% Simulation inputs
sim.Ts = 0.01;
sim.Tend = 1440;
Quad.Ts = sim.Ts; 
Quad.sim_time = sim.Tend;

%% Path Variables & References
Ref.pathType = 2;
Ref.start = [0; 0];
Ref.finish = [20; 0];
Ref.uRefNominal = 0.5;

%% Initialize Vehicle
quad_variables;
quad_dynamics_nonlinear;

vCorr = 0.0;

%% Run The Simulation Loop
for t = Quad.t_plot
    
    % Path Follower
    lookaheadPathFollowerUAV;
    
    % Coordination
    coordinationUAV(vCorr);
    
    % End condition
    if Quad.gamma >= complete
        % go to finish
        Quad.X_des_GF = Ref.finish(1,1);
        Quad.Y_des_GF = Ref.finish(2,1); 
        
        % update error term 
        e = sqrt((Quad.X_des_GF - Quad.X)^2 + (Quad.Y_des_GF - Quad.Y)^2);
        Quad.e_plot(Quad.counter) = e;
        
        % update reference for plot
        Quad.lookahead_plot(1,Quad.counter) = Quad.X_des_GF;
        Quad.lookahead_plot(2,Quad.counter) = Quad.Y_des_GF;

    end
    
    % Implement Controller
    position_PID;
    attitude_PID;
    rate_PID;
    
    % Calculate Desired Motor Speeds
    quad_motor_speed;
    
    % Update Position With The Equations of Motion
    quad_dynamics_nonlinear; 
    
    Quad.init = 1;  %Ends initialization after first simulation iteration    
end

%% Plots
% trajectory
figure('Name', 'Trajectory');
hold on;
grid on;
axis('equal');
plot3(Quad.lookahead_plot(1,:), Quad.lookahead_plot(2,:), Quad.Z_ref_plot, '--r');
plot3(Quad.X_plot, Quad.Y_plot, Quad.Z_plot);
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
hold off;

% Error Plot
% process error
uReal = sqrt(Quad.X_dot_plot.^2 + Quad.Y_dot_plot.^2);
uError = uReal - Ref.uRefNominal;

figure('Name', 'Error Terms');

% cross track error
subplot(2,1,1);
title('Cross Track Error');
hold on;
grid on;
plot(Quad.t_plot, 10^(3)*Quad.e_plot(1:length(Quad.t_plot)));
xlabel('time (s)');
ylabel('cross track error (mm)');
hold off

% velocity error
subplot(2,1,2);
title('Speed Error');
hold on;
grid on;
plot(Quad.t_plot, uError(1:length(Quad.t_plot)));
xlabel('time (s)');
ylabel('velocity error (m/s)');
hold off;