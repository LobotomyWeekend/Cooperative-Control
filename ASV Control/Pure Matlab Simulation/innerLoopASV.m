%% ASV INNER LOOP
% Distance in metres
% Angles in Radians
% Time in seconds

%% INNER LOOP FUNCTION
function [ASV] = innerLoopASV(ref, ASV, sim, i)
    %% CONTROLLERS
    % Heading Controller
    [headingCommand, ASV.yawIntHold] = headingController(ref.yawRef, ASV, sim);
    % Speed Controller
    [speedCommand, ASV.speedHold] = speedController(ref.uRef, ASV.state, ASV.speedHold);
    
    ASV.cmdHist(i).speedCommand = speedCommand;
    ASV.cmdHist(i).headingCommand = headingCommand;

    %% VEHICLE
    % Required Motor RPM from Heading + Speed Commands
    [RPMp, RPMs] = motorPower(headingCommand, speedCommand);
    ASV.RPM_Hist(1,i) = RPMp;
    ASV.RPM_Hist(2,i) = RPMs;
    
    ASV.cmdHist(i).RPMp = RPMp;
    ASV.cmdHist(i).RPMs = RPMs;
    
    % Thruster Moment from command RPM
    [tau_u, tau_r, Fs, Fp] = thrusters(RPMs, RPMp, ASV, sim, i);
    
    ASV.cmdHist(i).tau_r = tau_r;
    ASV.cmdHist(i).tau_u = tau_u;
    ASV.cmdHist(i).Fs = Fs;
    ASV.cmdHist(i).Fp = Fp;
    
    % Time derivitive of state from Dynamics
    [ASV] = vehicleDynamics(ASV, tau_u, tau_r);
    ASV.dStateHist(1,i) = ASV.dState;
    % Current state by integration
    ASV.state = estimateState(ASV, sim, i);

end % inner loop function