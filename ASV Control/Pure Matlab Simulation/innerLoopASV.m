%% ASV INNER LOOP
% Distance in metres
% Angles in Radians
% Time in seconds

%% INNER LOOP FUNCTION
function [ASV] = innerLoopASV(ref, ASV, sim, i)
    %% CONTROLLERS
    % Heading Controller
    headingCommand = headingController(ref.yawRef, ASV.state);
    % Speed Controller
    [speedCommand, ASV.speedHold] = speedController(ref.uRef, ASV.state, ASV.speedHold);

    %% VEHICLE
    % Required Motor RPM from Heading + Speed Commands
    [RPMp, RPMs] = motorPower(headingCommand, speedCommand);
    ASV.RPM_Hist(1,i) = RPMp;
    ASV.RPM_Hist(2,i) = RPMs;
    % Thruster Moment from command RPM
    [tau_u, tau_r] = thrusters(RPMs, RPMp, ASV, sim, i);
    % Time derivitive of state from Dynamics
    [ASV] = vehicleDynamics(ASV, tau_u, tau_r);
    ASV.dStateHist(1,i) = ASV.dState;
    % Current state by integration
    ASV.state = integrateState(ASV.dStateHist, sim, i);

end % inner loop function