function [yawRef, ASV] = straightLinePath(ASV, ref)
%% Commands an ASV to follow a path
% Takes a start and and end point of a straight line path, and controls the
% yaw of the ASV in order for it to follow the path. This is done by
% calculating the cross track translational error, as well as the error in
% yaw, and updating the yaw reference with a PI controller.

    %% Process Path
    % find gradient, y offset, and yaw angle of a straight line
    [m, c, yawD] = processLine(ref.start,ref.finish);

    %% Point on the path nearest the vehicle's current location
    % adjusting for sigularity in equation
    if m == -1 
        xD = (ASV.X - ASV.Y)/2;
        yD = m*xD + c;
        
    % vertical path (i.e. infinite gradient)    
    elseif abs(m) == Inf 
        xD = ref.start(1,1);
        yD = ASV.Y;
      
    % Standard case
    else 
        xD = (ASV.X + ASV.Y - c)/(m + 1);
        yD = m*xD + c;
    end

    %% Cross Track Error
    % absolute value of error
    crossTrack = sqrt((xD - ASV.X)^2 + (yD - ASV.Y)^2);

    % Clockwise from path is defined as negative crossTrack error, i.e. yaw
    % needs to be INCREASED
    
    % Check whether line with infinite gradient x = const is supplied, and
    % correct for singularities in equations used.
    if abs(m) ~= Inf % standard case
        path = m*ASV.X + c;
        
        % +ve or -ve cross track
        if ASV.Y < path
            crossTrack = - crossTrack;
        end
    % upwards path x {G} = const, y_dot {G} = +ve    
    elseif m == Inf && ASV.X > ref.start(1,1) 
        crossTrack = - crossTrack;
    
    % downawrds path x {G} = const, y_dot {G} = -ve    
    elseif m == -Inf && ASV.X < ref.start(1,1)
        crossTrack = - crossTrack;
    end

    % save to vehicle struct
    ASV.error_crossTrack = crossTrack;

    %% Integral of crossTrack Error
    % save to vehicle struct
    ASV.error_crossTrack_int = ASV.error_crossTrack_int + ASV.error_crossTrack * ASV.Ts;

    %% Proportional Error in Yaw
    % save to vehicle struct
    ASV.error_yaw = yawD - ASV.Yaw;

    %% Control Algorithm
    % gain values
    K1 =  10.0; %yaw proportional
    K2 =  5.0; %cross-track proportional
    K4 =  0.2; %integral
    
    % adjust for positive or negative yaw direction
    if ASV.X > 0
        direc = -1;
    else
        direc = 1;
    end

    % control term
    yawDel = K1*ASV.error_yaw + direc * K2 * crossTrack / ref.uRef ...
             + direc * K4 * ASV.error_crossTrack_int;
         
    % update yaw reference
    yawRef = yawD + yawDel;
    
    %% Coordination state
    % total path length
    L    = sqrt((ref.finish(1,1) - ref.start(1,1))^2 +(ref.finish(2,1) - ref.start(2,1))^2);
    
    % portion of path covered
    Lpos = sqrt((ASV.X - ref.start(1,1))^2 +(ASV.Y - ref.start(2,1))^2);
    
    % gamma defined as fraction of total path length in range [0,1]
    ASV.gamma = Lpos/L;
    
end
