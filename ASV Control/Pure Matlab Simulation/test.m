%% TESTING ASV INNER LOOP
% function which mimics the behaviour of the simulink model
% i.e. can be placed inside a time loop with ref values changed
% dynamically.
clear all;
close all;

%% Simulation Inputs
% time
sim.Ts   = 0.1;
sim.Tend = 360;
sim.time = 0:sim.Ts:sim.Tend;

% waypoints
ref.start = [0;0];
ref.finish = [0;0];

%% Initialize Vehicles
% vehicle 1
yawInit = 0;
yawFinal = 90;
% ASV sctruct
ASV1 = initializeASV(ref, sim);
% initial conditions
[ASV1.IC, ASV.state] = initialConditions(ref, yawInit);


%% Simulation
i = 1;
for t = sim.time
    %% Calculate References
    if t < 10
        ref.yawRef = yawInit;
    else
        ref.yawRef = yawFinal;
    end
    ref.uRef = 1;

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
% references and response
plotRefValues(ASV1, ref, sim);
% trajectory
figure('Name', 'Trajectory');
hold on; grid on;
axis('equal');
plot([ASV1.stateHist.x],[ASV1.stateHist.y]);
xlabel('x (m)');
ylabel('y (m)');
hold off;

