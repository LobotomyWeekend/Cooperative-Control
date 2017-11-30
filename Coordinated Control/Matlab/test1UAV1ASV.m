%% Initialize Workspace
clear all;
close all;
clc;

% constants
complete = 1.0;

%% Simulation inputs
sim.Ts = 0.01;
sim.Tend = 360;
sim.time = 0:sim.Ts:sim.Tend;

Quad.Ts = sim.Ts; 
Quad.sim_time = sim.Tend;

%% UAV Setup
% waypoints
ref1.pathType = 2;
ref1.start = [0; 0];
ref1.finish = [30; 0];
% constant speed reference
ref1.uRefNominal = 0.5;
% initialize vehicle structure
UAV1 = quad_variables(sim, 1, ref1.start);
UAV1 = quad_dynamics_nonlinear(UAV1);

%% ASV Setup
% waypoints
ref2.pathType = 2;
ref2.start = [5;0];
ref2.finish = [25;10];
% constant speed reference
ref2.uRefNominal = 0.5;
% initial values
yawInit = 0;
% initialize vehicle structure
ASV1 = ASV_variables(sim, ref1.start, yawInit, 2);


%% SIMULATION
for t =  sim.time
    % Coordination
    vCorr = coordinationMaster(UAV1, ASV1);
    ref2.uRef = ref2.uRefNominal + vCorr(ASV1.vehicleID);
    
    % Path Following
    UAV1 = lookaheadPathFollowerUAV(UAV1, ref1, vCorr(UAV1.vehicleID));
    [ref2.yawRef, ASV1] = pathFollowerASV(ASV1, ref2);

%     % End condition
%     UAV1 = endConditionUAV(UAV1, ref1, complete);
%     ref2 = endCnditionASV(ASV1, ref2, complete);
    
    % Inner Loop Dynamics and Controllers
    UAV1 = innerLoopUAV(UAV1);
    ASV1 = innerLoopASV(ref2, ASV1);

    Quad.init = 1;
end

%% Plotting
figure('Name','Trajectory');
hold on;
grid on;
plot(UAV1.X_plot, UAV1.Y_plot);
plot(ASV1.X_plot, ASV1.Y_plot);
hold off;

figure('Name','Coordination States');
hold on;
grid on;
plot(sim.time, UAV1.gamma_plot);
plot(sim.time, ASV1.gamma_plot);
legend('UAV1','ASV1');
hold off;