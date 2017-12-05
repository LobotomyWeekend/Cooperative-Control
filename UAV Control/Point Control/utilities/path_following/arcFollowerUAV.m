function Quad = arcFollowerUAV(Quad, Ref, uRef, rotationalDirection)
    %% Arc Path Following for UAV
    % gain terms
    K2 = 2.5; % proportional cross track
    
    % process arc
    [xM, yM, r, ~] = processArc(Ref.start,Ref.finish);

    % spoof location if circle was centred on [0,0]
    xSpoof = Quad.X - xM;
    ySpoof = Quad.Y - yM;

    % angular position around circle
    Quad.theta_pos = atan2d(ySpoof,xSpoof);

    % closest point on path
    xD = r*cosd(Quad.theta_pos);
    yD = r*sind(Quad.theta_pos);

    %% Coordination State & Desired Yaw
    % gamma       : theta_pos as percentage total
    % yaw_desired : angle of the current tangent
    
    if rotationalDirection == "CW"
        
        Quad.gamma = (180 - Quad.theta_pos)/180;
        Quad.yaw_desired = Quad.theta_pos - 90;
        
    elseif rotationalDirection == "CCW"
        
        Quad.gamma = (180 + Quad.theta_pos)/180;
        Quad.yaw_desired = Quad.theta_pos + 90;
        
    else
        
        error('Invalid Rotational Direction Argument UAV');
        
    end

    %% Cross track error control
    % cross track error
    e = sqrt((xD - xSpoof)^2 + (yD - ySpoof)^2);

    % update position command
    if sqrt(xSpoof^2 + ySpoof^2) > r
        % outside path
        r = r - K2 * e;
    else
        % inside path
        r = r + K2 * e;
    end

    %% Update reference
    % change in theta each timestep (clockwise)
    dTheta = uRef/r*Quad.Ts*180/pi;
    
    % counterclockwise
    if rotationalDirection == "CCW"
        dTheta = - dTheta;
    end

    % update desired position
    Quad.X_des_GF = r*(cosd(Quad.theta_pos - dTheta)) + xM;
    Quad.Y_des_GF = r*(sind(Quad.theta_pos - dTheta)) + yM;  
    
    %% Store History
    % history of lookahead points
    Quad.lookahead_plot(:,Quad.counter) = [Quad.X_des_GF; Quad.Y_des_GF];
    
    % history of cross track error
    Quad.error_crossTrack_plot(Quad.counter) = e;
    
    Quad.thetaGuess_plot(Quad.counter) = Quad.theta_pos;
    Quad.desired_plot(:,Quad.counter) = [xD; yD];

end