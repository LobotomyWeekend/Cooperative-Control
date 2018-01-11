function ASV = innerLoopASV(ref, ASV)
%% ASV INNER LOOP
% Takes references in speed and heading provided by outer loop controllers,
% and commands the states, and calculates the response, of the vehicle
% This should be called each simulation loop

    %% STATE CONTROLLERS
    % Heading Controller
    ASV = headingController(ref.yawRef, ASV);
    % Speed Controller
    ASV = speedController(ASV, ref.uRef);

    %% VEHICLE
    % Required Motor RPM from Heading + Speed Commands
    ASV = motorPower(ASV);
    % Resultant force and moment
    ASV = thrusters(ASV);
    % Vehicle Dynamics
    ASV = vehicleDynamics(ASV);
    % Update the state variables
    ASV = estimateState(ASV);

end