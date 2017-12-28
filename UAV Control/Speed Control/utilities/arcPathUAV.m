function [yawRef, UAV] = arcPathUAV(UAV, ref)
    %% ARC PATH for ASV
    % Takes a start point and end point and commands the vehicle to follow
    % an arc between these points. Achieved by making the yawRef = tangent
    % angle of nearest point on path, forwards momentum allows for
    % progression of this command variable.
    
    %% Workspace
    persistent error_crossTrack_int;
    
    if UAV.counter == 1
        error_crossTrack_int = 0;
    end
    
    %% Process Path    
    % find arc properties
    [xM, yM, r, ~] = processArc(ref.start,ref.finish);

    % spoof location if circle was centred on [0,0]
    xSpoof = real(UAV.X) - xM;
    ySpoof = real(UAV.Y) - yM;
    
    % angular progression around circle
    theta = atan2d(ySpoof,xSpoof);
    
    % find desired yaw
    if ref.pathType == 2 % clockwise arc
        yawD = theta - 90;
    elseif ref.pathType == 3 % counter clockwise arc
        yawD = theta + 90;
    end
    
    %% Error
    % cross track error
    xD = r*cosd(theta);
    yD = r*sind(theta);
    UAV.error_crossTrack = sqrt((xD - xSpoof)^2 + (yD - ySpoof)^2);
    
    % check if above or below arc
    if UAV.Y < yD
        % below, increase yaw
        UAV.error_crossTrack = -UAV.error_crossTrack;
    end
    
    % calculate heading (not equivalent to yaw in UAV)
    heading = atan2d(UAV.Y_dot, UAV.X_dot);
    
    % yaw error
    UAV.error_yaw = yawD - heading;
    
    %% Integral
    error_crossTrack_int = error_crossTrack_int + UAV.error_crossTrack * UAV.Ts;   
    
    %% Provide Reference
    K1 = 0.1; % proportional yaw error
    K2 = - 100;  % proportional cross track error
    K3 = - 10; %integral cross 
    
    yawRef = yawD + K1 * UAV.error_yaw + K2 * UAV.error_crossTrack + K3 * error_crossTrack_int;
    
    
    %% Coordination state
    if ref.pathType == 2 % clockwise arc
        UAV.gamma = (180 - theta) / 180;
    elseif ref.pathType == 3 % counter clockwise arc
        UAV.gamma = (180 + theta) / 180;
    end
    
end
