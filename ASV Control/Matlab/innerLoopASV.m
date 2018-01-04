
%% ASV INNER LOOP
% Distance in metres
% Angles in Radians
% Time in seconds

%% INNER LOOP FUNCTION
function ASV = innerLoopASV(ref, ASV)
    %% CONTROLLERS
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

end % inner loop function