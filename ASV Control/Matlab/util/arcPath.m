function [yawRef, ASV] = arcPath(ASV, ref)
    %% ARC PATH for ASV
    % Takes a start point and end point and commands the vehicle to follow
    % an arc between these points. Achieved by making the yawRef = tangent
    % angle of nearest point on path, forwards momentum allows for
    % progression of this command variable.
    
    %% Process Path    
    % find arc properties
    [xM, yM, r, ~] = processArc(ref.start,ref.finish);

    % spoof location if circle was centred on [0,0]
    xSpoof = real(ASV.X) - xM;
    ySpoof = real(ASV.Y) - yM;
    
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
    ASV.error_crossTrack = sqrt((xD - xSpoof)^2 + (yD - ySpoof)^2);
    
    % check if above or below arc
    if ASV.Y < yD
        % below, increase yaw
        ASV.error_crossTrack = -ASV.error_crossTrack;
    end
    
    % yaw error
    ASV.error_yaw = yawD - ASV.Yaw;
    
    %% Integral
    ASV.error_crossTrack_int = ASV.error_crossTrack_int + ASV.error_crossTrack * ASV.Ts;    
    
    %% Provide Reference
    K1 = 75; % proportional yaw error
    K2 = 750;  % proportional cross track error
    K3 = 10; %integral cross 
    
    yawRef = yawD + K1 * ASV.error_yaw - K2 * ASV.error_crossTrack - K3 * ASV.error_crossTrack_int;
    
    %% Coordination state
    if ref.pathType == 2 % clockwise arc
        ASV.gamma = (180 - theta) / 180;
    elseif ref.pathType == 3 % counter clockwise arc
        ASV.gamma = (180 + theta) / 180;
    end
    
    
end
