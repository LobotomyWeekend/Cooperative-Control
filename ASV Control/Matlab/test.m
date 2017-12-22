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
ref.finish = [0;10];

% Initial Reference
ref.uRef = 0;
ref.yawRef = 0;

%% Initialize Vehicles
% initial yaw value
yawInit = 0; 
% establish structure
ASV1 = ASV_variables(sim, ref.start, yawInit, 1);

%% Constants
complete = 1;
sign = 1;

%% Preallocate matrices
uRefHist = zeros(1,length(sim.time));
uHist = uRefHist;

%% Simulation
i = 1;
for t = sim.time
    %% Calculate References
    % Stepping yawRef
    if ~mod(t,25)
        % Update yaw reference
        ref.uRef = ref.uRef + 0.2 * sign;
        % Flip sign at yawRef = 120
        if ref.uRef >= 1 && sign == 1
            sign = -1;
        end
        % Prevent from going below zero
        if ref.uRef <= 0.01 && sign == -1
            sign = 0;
        end
    end

    %% Simulate Vehicles
    % ASV 1
    [ASV1] = innerLoopASV(ref, ASV1);
    
    %% Save Yaw History
    uRefHist(i) = ref.uRef;
    if ASV1.X_dot >= 0
        U = sqrt(ASV1.X_dot^2 + ASV1.Y_dot^2);
    else
        U = - sqrt(ASV1.X_dot^2 + ASV1.Y_dot^2);
    end
    uHist(i) = U;
    
    i = i+1;
end

%% Plotting
hold on
grid on;
title('Velocity Response ASV');
plot(sim.time, uRefHist, '--');
plot(sim.time, uHist);
xlabel('time(s)');
ylabel('Velocity (m/s)');
legend('Velocity Ref', 'ASV Velocity');
hold off
