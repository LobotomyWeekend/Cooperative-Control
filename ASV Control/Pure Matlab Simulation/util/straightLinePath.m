function [yawRef, ASV] = straightLinePath(ASV, ref, sim, i)      
    %% Process Path
    [m, c, yawD] = processLine(ref.start,ref.finish);

    %% Path Error
    % find nearest point's x
    if m == -1
        % adjusting for sigularity in xD function
        xD = (ASV.state.x - ASV.state.y)/2;
    else
        % standard case
        xD = (ASV.state.x + ASV.state.y - c)/(m + 1);
    end

    % find nearest point's y
    yD = m*xD + c;

    closestPoint = [xD;yD];

    % find cross track error
    crossTrack = sqrt((xD - ASV.state.x)^2 + (yD - ASV.state.y)^2);

    path = m*ASV.state.x + c;
    if ASV.state.y < path
        crossTrack = - crossTrack;
    end

    ASV.error.e = crossTrack;

    %% Integral
    if i == 1
        ASV.error.eIntHold = 0;
    end
    ASV.error.eInt = ASV.error.eIntHold + ASV.error.e*sim.Ts;
    ASV.error.eIntHold = ASV.error.eInt;

    %% Yaw error
    ASV.error.yaw = yawD - ASV.state.yaw;

    %% Provide Yaw Ref
    % gain values
    K1 =  10.0; %yaw proportional
    K2 =  5.0; %cross-track proportional
    K4 =  0.2; %integral

    if ASV.state.x > 0
        direc = -1;
    else
        direc = 1;
    end

    % delta term
    yawDel = K1*ASV.error.yaw + direc*K2*crossTrack/ref.uRef ...
             + direc*K4*ASV.error.eInt;
    yawRef = yawD + yawDel;
    
    %% Coordination state
    L    = sqrt((ref.finish(1,1) - ref.start(1,1))^2 +(ref.finish(2,1) - ref.start(2,1))^2);
    Lpos = sqrt((ASV.state.x - ref.start(1,1))^2 +(ASV.state.y - ref.start(2,1))^2);
    ASV.coOrd.gamma = Lpos/L;
end
