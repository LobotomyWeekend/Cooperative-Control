%% Speed Controller
function [speedCommand, qsi] = speedController(uRef, state, qsiold)
    % Gain Values
    Kp = 60;
    Ki = 2;
    KSat = 60;

    % Absolute Velocity
    absVel = sign(state.u)*sqrt((state.u*cosd(state.yaw))^2 + (state.v*sind(state.yaw))^2);
    % Calculate Error
    err = absVel - uRef;
    qsi = qsiold + err;
    
    % Speed command w saturation [-KSat,KSat]
    speedCommand = - Kp*err - Ki*qsi;
    if(abs(speedCommand) >= KSat)
       speedCommand = KSat*sign(speedCommand); 
       qsi = qsiold;
    end
    
    % Squared output (maintain sign)
    speedCommand = abs(speedCommand)*speedCommand;
end
