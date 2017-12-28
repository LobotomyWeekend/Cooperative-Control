function [yawRef, UAV] = straightLinePathUAV(UAV, ref)
%% Commands an ASV to follow a path
% Takes a start and and end point of a straight line path, and controls the
% yaw of the ASV in order for it to follow the path. This is done by
% calculating the cross track translational error, as well as the error in
% yaw, and updating the yaw reference with a PI controller.

    %% Workspace
    persistent error_crossTrack_int;
    
    if UAV.counter == 1
        error_crossTrack_int = 0;
    end
    
    %% Process Path
    % find gradient, y offset, and yaw angle of a straight line
    [m, c, yawD] = processLine(ref.start,ref.finish);

    %% Point on the path nearest the vehicle's current location
    % adjusting for sigularity in equation
    if m == -1 
        xD = (UAV.X - UAV.Y)/2;
        yD = m*xD + c;
        
    % vertical path (i.e. infinite gradient)    
    elseif abs(m) == Inf 
        xD = ref.start(1,1);
        yD = UAV.Y;
      
    % Standard case
    else 
        xD = (UAV.X + UAV.Y - c)/(m + 1);
        yD = m*xD + c;
    end

    %% Cross Track Error
    % absolute value of error
    crossTrack = sqrt((xD - UAV.X)^2 + (yD - UAV.Y)^2);

    % Clockwise from path in defined as negative crossTrack error
    
    if m ~= Inf % standard case
        path = m*UAV.X + c;
        if UAV.Y < path
            crossTrack = - crossTrack;
        end
        
    elseif m == Inf && UAV.X > ref.start(1,1) % upwards path
        crossTrack = - crossTrack;
        
    elseif m == -Inf && UAV.X < ref.start(1,1) % downwards path
        crossTrack = - crossTrack;
    end

    % save to vehicle struct
    UAV.error_crossTrack = crossTrack;

    %% Integral of crossTrack Error
    % save to vehicle struct
    error_crossTrack_int = error_crossTrack_int + crossTrack * UAV.Ts;

    %% Proportional Error in Yaw
    heading = atan2d(UAV.Y_dot, UAV.X_dot);
    if heading < 0
        heading = heading + 360;
    end
    % save to vehicle struct
    UAV.error_yaw = yawD - heading;

    %% Control Algorithm
    % gain values
    K1 =  1.0; %yaw proportional
    K2 =  -1.5; %cross-track proportional
    K4 =  -1.0; %integral
    
    % control term
    yawDel = K1*UAV.error_yaw + K2 * crossTrack / ref.uRef ...
             + K4 * error_crossTrack_int;
         
    % update yaw reference
    yawRef = yawD + yawDel;
    
    %% Coordination state
    % total path length
    L    = sqrt((ref.finish(1,1) - ref.start(1,1))^2 +(ref.finish(2,1) - ref.start(2,1))^2);
    
    % portion of path covered
    Lpos = sqrt((UAV.X - ref.start(1,1))^2 +(UAV.Y - ref.start(2,1))^2);
    
    % gamma defined as fraction of total path length in range [0,1]
    UAV.gamma = Lpos/L;
    
end
