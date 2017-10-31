%% TESTING ASV INNER LOOP
% function which mimics the behaviour of the simulink model
% i.e. can be placed inside a time loop with ref values changed
% dynamically.
clear all;

%% Simulation Inputs
% time
sim.Ts   = 0.2;
sim.Tend = 120;
sim.time = 0:sim.Ts:sim.Tend;
% waypoints
ref.start = [0;0];
ref.finish = [10;10];

%% Initialize Vehicles
% vehicle 1
ASV1 = initializeASV(ref, sim);

%% Simulation
i = 1;
for t = sim.time
    %% Calculate References
    ref.uRef = 1;
    [ref, ASV1] = pathFollowerASV(ASV1, ref, sim, i);

    %% Simulate Vehicles
    % ASV 1
    [ASV1] = innerLoopASV(ref, ASV1, sim, i);

    %% Save Data
    % state history
    ASV1.stateHist(i) = ASV1.state;
    ref.refHist.yawRef(i) = ref.yawRef;
    ref.refHist.uRef(i) = ref.uRef;
    i = i + 1;

end

%% Plots
close all;
% references and response
plotRefValues(ASV1, ref, sim);
% trajectory
figure('Name', 'Trajectory');
hold on; grid on;
plot([ASV1.stateHist.x],[ASV1.stateHist.y]);
xlabel('y (m)');
ylabel('x (m)');
hold off;

