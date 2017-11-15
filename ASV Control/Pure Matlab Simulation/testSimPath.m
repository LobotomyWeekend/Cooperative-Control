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
yawInit = 0;
% ASV sctruct
ASV1 = initializeASV(ref, sim);
% initial conditions
[ASV1.IC, ASV.state] = initialConditions(ref, yawInit);

%% Simulation
i = 1;
for t = sim.time
    %% Calculate References
    ref.uRef = 1;
    % [ref, ASV1] = pathFollowerASV(ASV1, ref, sim, i);
    ref.yawRef = 45;

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
    % reference
    xlim = [min([ASV1.stateHist.x]), max([ASV1.stateHist.x])];
    A = ref.start;
    B = ref.finish;
    m = (B(2)-A(2))/(B(1)-A(1));
    n = A(2) - m*A(1);
    y1 = m*xlim(1) + n;
    y2 = m*xlim(2) + n;
    line([xlim(1) xlim(2)],[y1 y2], 'Color','red','LineStyle','--');
xlabel('y (m)');
ylabel('x (m)');
hold off;

