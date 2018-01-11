%% Comparison of Quadrotor controller with or without disturbance
% This simulates two UAVs following a simple path with constant yaw
% reference, one with a constant disturbance and the other without.

clear all;
close all;
clc;

%% Initialize Workspace
% constants
complete = 1.0;
vCorr = 0.0;

%% Simulation inputs
sim.Ts = 0.01;
sim.Tend = 200;

%% Path Variables & References
ref1.pathType = 1;
ref1.start = [0; 0]; % also vehicle's initial position
ref1.finish = [200; 200];
ref1.uRefNominal = 0.5;
ref1.uRef = 0.5;

ref2.pathType = 1;
ref2.start = [0;0];
ref2.finish = [200;200];
ref2.uRefNominal = 0.5;
ref2.uRef = 0.5;

ref3.pathType = 1;
ref3.start = [0; 0]; % also vehicle's initial position
ref3.finish = [200; 200];
ref3.uRefNominal = 0.5;
ref3.uRef = 0.5;

ref4.pathType = 1;
ref4.start = [0; 0]; % also vehicle's initial position
ref4.finish = [200; 200];
ref4.uRefNominal = 0.5;
ref4.uRef = 0.5;

yawRef = 45;

%% Initialize Vehicles
UAV1 = quad_variables(sim, 1, ref1.start);
UAV1 = quad_dynamics_nonlinear(UAV1);
UAV1.ref = ref1;

UAV2 = quad_variables(sim, 2, ref2.start);
UAV2.X_dis = 0.25;
UAV2 = quad_dynamics_nonlinear(UAV2);
UAV2.ref = ref2;

UAV3 = quad_variables(sim, 3, ref3.start);
UAV3.X_dis = 0.5;
UAV3 = quad_dynamics_nonlinear(UAV3);
UAV3.ref = ref3;

UAV4 = quad_variables(sim, 4, ref4.start);
UAV4.X_dis = 0.75;
UAV4 = quad_dynamics_nonlinear(UAV4);
UAV4.ref = ref4;


%% Run The Simulation Loop
for t = UAV1.t_plot
    % Display Progression
    displayProgress(UAV1);
    
    % Path Follower
    UAV1 = parameterizeSpeed(UAV1, yawRef);
    UAV2 = parameterizeSpeed(UAV2, yawRef);
    UAV3 = parameterizeSpeed(UAV3, yawRef);
    UAV4 = parameterizeSpeed(UAV4, yawRef);
    
    % Inner Loop Dynamics and Controllers
    UAV1 = innerLoopUAV(UAV1);
    UAV2 = innerLoopUAV(UAV2);
    UAV3 = innerLoopUAV(UAV3);
    UAV4 = innerLoopUAV(UAV4);
end
clc
disp('Finishing Up...');

%% Plots
% trajectory
plotTrajectory(UAV1, UAV2, UAV3, UAV4);
% coordination state
% plotCoordination(UAV1, UAV2, UAV3);
% cross track error
% plotCrossTrackError(UAV1, UAV2, UAV3, UAV4);
% control terms
PlotUAVCommands(UAV1, "Speed");
PlotUAVCommands(UAV2, "Speed");
PlotUAVCommands(UAV3, "Speed");
PlotUAVCommands(UAV4, "Speed");


clc;