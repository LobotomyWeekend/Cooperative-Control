%% Speed Controller
function [speedCommand, qsi] = speedController(uRef, state, qsiold)
    % Gain Values
    Kp = 60;
    Ki = 2;
    KSat = 60;

    % Absolute Velocity
    absVel = state.u*sin(state.yaw) + state.v*cos(state.yaw);
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
