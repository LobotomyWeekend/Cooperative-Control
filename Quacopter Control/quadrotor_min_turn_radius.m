%% Simulates UAV-UAV Cooperation
% Two UAVs working in coordination for any suitable path type, plots
% trajectory, cross track error, and coordination states over time.

%% Initialize Workspace
clear all;
close all;
clc;

% constants
complete = 1.0;

%% Simulation inputs
sim.Ts = 0.01;
sim.Tend = 60;
sim.t = 0:sim.Ts:sim.Tend;

%% Path Variables & References
% Vehicle 1 % d = 10
ref1.pathType = 2;
ref1.start = [0; 0];
ref1.finish = [4; 0];
ref1.uRefNominal = 0.5;
ref1.waypoints = ref_waypoints(ref1);
% Vehicle 2 % d = 7
ref2.pathType = 2;
ref2.start = [0; 0];
ref2.finish = [3; 0];
ref2.uRefNominal = 0.5;
ref2.waypoints = ref_waypoints(ref2);
% Vehicle 3 % d = 4
ref3.pathType = 2;
ref3.start = [0; 0];
ref3.finish = [2; 0];
ref3.uRefNominal = 0.5;
ref3.waypoints = ref_waypoints(ref3);
% Vehicle 4 % d = 1
ref4.pathType = 2;
ref4.start = [0; 0];
ref4.finish = [1; 0];
ref4.uRefNominal = 0.5;
ref4.waypoints = ref_waypoints(ref4);

%% Initialize Vehicles
% UAV1
UAV1 = quad_variables(sim, 1, ref1.start);
UAV1 = quad_dynamics_nonlinear(UAV1);
% UAV2
UAV2 = quad_variables(sim, 2, ref2.start);
UAV2 = quad_dynamics_nonlinear(UAV2);
% UAV3
UAV3 = quad_variables(sim, 3, ref3.start);
UAV3 = quad_dynamics_nonlinear(UAV3);
% UAV4
UAV4 = quad_variables(sim, 4, ref4.start);
UAV4 = quad_dynamics_nonlinear(UAV4);

UAV1.ref = ref1;
UAV2.ref = ref2;
UAV3.ref = ref3;
UAV4.ref = ref4;

vCorr = [ 0 0 0 0 ];

%% Run The Simulation Loop
i = 1;
for t = sim.t
    %% Simulation
    % Display Progression
    displayProgress(UAV1);
    
%     % Master coordination controller
%     vCorr = coordinationMaster(UAV1, UAV2, UAV3, UAV4);
    
    % Update speed reference
    UAV1.ref.uRef = ref1.uRefNominal + vCorr(1,1);
    UAV2.ref.uRef = ref2.uRefNominal + vCorr(1,2);
    UAV3.ref.uRef = ref3.uRefNominal + vCorr(1,3);
    UAV4.ref.uRef = ref4.uRefNominal + vCorr(1,4);
    
    % Path Follower
    UAV1 = pathFollowerUAV(UAV1, ref1);
    UAV2 = pathFollowerUAV(UAV2, ref2);
    UAV3 = pathFollowerUAV(UAV3, ref3);
    UAV4 = pathFollowerUAV(UAV4, ref4);
    
    % Inner Loop Dynamics and Controllers
    UAV1 = innerLoopUAV(UAV1);
    UAV2 = innerLoopUAV(UAV2);
    UAV3 = innerLoopUAV(UAV3);
    UAV4 = innerLoopUAV(UAV4);
    
    % End condition
    UAV1 = endConditionUAV(UAV1);
    UAV2 = endConditionUAV(UAV2);
    UAV3 = endConditionUAV(UAV3);
    UAV4 = endConditionUAV(UAV4);
    
    vCorr_hist(:,i) = vCorr;
    i = i + 1;
end
clc
disp('Finishing Up...');

% maximum cross track error
e_max1 = max(UAV1.error_crossTrack_plot);
e_max2 = max(UAV2.error_crossTrack_plot);
e_max3 = max(UAV3.error_crossTrack_plot);
e_max4 = max(UAV4.error_crossTrack_plot);
% rms cross track error
e_rms1 = rms(UAV1.error_crossTrack_plot);
e_rms2 = rms(UAV2.error_crossTrack_plot);
e_rms3 = rms(UAV3.error_crossTrack_plot);
e_rms4 = rms(UAV4.error_crossTrack_plot);


clc