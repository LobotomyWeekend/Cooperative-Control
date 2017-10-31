%% SIMULATION OF MEDUSA ASV INNER LOOP
% Distance in metres
% Angles in Radians
% Time in seconds

%% INNER LOOP FUNCTION
function [stateHist,dStateHist] = innerLoopASV(yawRef,uRef, Ts, Tend)
    %% Time Variable
    time = 0:Ts:Tend;

    %% Initial Conditions
    [state, dState] = initializeStuct();

    % Preallocation etc.
    stateHist  = state;
    dStateHist = dState;
    RPM_Hist   = zeros(2 , length(time));
    holdSpeedValue = 0;
    i = 1;

    %% Implementation
    % Loop over time
    for t = time
        %% CONTROLLERS
        % Heading Controller
        headingCommand = headingController(yawRef,state);
        % Speed Controller
        [speedCommand, holdSpeedValue] = speedController(uRef, state, holdSpeedValue);

        %% VEHICLE
        % Required Motor RPM from Heading + Speed Commands
        [RPMp, RPMs] = motorPower(headingCommand,speedCommand);
        % Thruster Moment from command RPM
        [tau_u, tau_r] = thrusters(RPMs, RPMp, RPM_Hist, t, Ts);
        % Time derivitive of state from Dynamics
        dState = vehicleDynamics(state, tau_u, tau_r);
        dStateHist(1,i) = dState;
        % Current state by integration
        state = integrateState(dStateHist, Ts, t);

        %% DATA
        % Save state history
        stateHist (i) = state;
        RPM_Hist(1,i) = RPMp;
        RPM_Hist(2,i) = RPMs;
        i = i + 1;
    end % loop over time
end % inner loop function