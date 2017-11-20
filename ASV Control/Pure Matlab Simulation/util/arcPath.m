function [yawRef, ASV] = arcPath(ASV, ref, sim, i)
    %% ARC PATH for ASV
    % Takes a start point and end point and commands the vehicle to follow
    % an arc between these points. Achieved by making the yawRef = tangent
    % angle of nearest point on path, forwards momentum allows for
    % progression of this command variable.
    
    %% Process Path
    % extract input data
    start  = ref.start;
    finish = ref.finish;
    
    % find midpoint
    xM = (start(1,1) + finish(1,1))/2;
    yM = (start(2,1) + finish(2,1))/2;
    
    % find radius
    r  = sqrt((finish(1,1) - xM)^2 + (finish(2,1) - yM)^2);
    
    % find gradient
    m  = (finish(2,1) - start(2,1)) / (finish(1,1) - start(1,1));
    gradAngle = atand(m);

    % spoof location if circle was centred on [0,0]
    xSpoof = ASV.state.x - xM;
    ySpoof = ASV.state.y - yM;
    
    % angular progression around circle
    theta = atan2d(ySpoof,xSpoof);
    
    % find desired yaw
    yawD = theta - 90;

    
    %% Error
    % cross track error
    xD = r*cosd(theta);
    yD = r*sind(theta);
    ASV.error.e = sqrt((xD - xSpoof)^2 + (yD - ySpoof)^2);
    
    % check if above or below arc
    if ASV.state.y < yD
        % below, increase yaw
        ASV.error.e = -ASV.error.e;
    end
    
    % yaw error
    ASV.error.yaw = yawD - ASV.state.yaw;
    
    %% Integral
    if i == 1
        ASV.error.eIntHold = 0;
    end
    ASV.error.eInt = ASV.error.eIntHold + ASV.error.e*sim.Ts;
    ASV.error.eIntHold = ASV.error.eInt;
    
    
    %% Provide Reference
    K1 = 75; % proportional yaw error
    K2 = 750;  % proportional cross track error
    K3 = 10; %integral cross 
    
    yawRef = yawD + K1*ASV.error.yaw - K2*ASV.error.e - K3*ASV.error.eInt;
    
    
end
