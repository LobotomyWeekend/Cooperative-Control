function Quad = lookaheadPathFollowerUAV(Quad, Ref)
%% Setup
% gain terms
K1 = 10; % proportional speed
K2 = 2.5; % proportional cross track


%% Speed Control
uReal = sqrt(Quad.X_dot^2 + Quad.Y_dot^2); % current absolute velocity
uError = uReal - Ref.uRefNominal; % error
uRef = Ref.uRefNominal - K1*uError; % new reference velocity

%% Path Following
% line = 1 / arc = 2
switch Ref.pathType
    case 1
        %% Straight Line Path Following
        % find path variables    
        [Ref.m, Ref.c, Ref.yawD] = processLine(Ref.start,Ref.finish);

        % find nearest point on line
        xPath = (Quad.X + Quad.Y - Ref.c)/(Ref.m + 1);
        yPath = Ref.m*xPath + Ref.c;

        % cross track error
        e = sqrt((xPath - Quad.X)^2 + (yPath - Quad.Y)^2);

        % Calculate incrememts
        dx = sqrt(uRef^2 / ((tand(Ref.yawD))^2 + 1)) * Quad.Ts;
        dy = sqrt(uRef^2 * (1 - 1/((tand(Ref.yawD))^1 + 1))) * Quad.Ts;

        % update desired position
        Quad.X_des_GF = 1/Ref.m*(yPath + dy - Ref.c);
        Quad.Y_des_GF = Ref.m*(xPath + dx) + Ref.c; 
    
    case 2
        %% Arc Path Following
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
        % change in theta each timestep
        dTheta = uRef/r*Quad.Ts*180/pi;
        
        % update desired position
        Quad.X_des_GF = r*(cosd(Quad.theta_pos - dTheta)) + xM;
        Quad.Y_des_GF = r*(sind(Quad.theta_pos - dTheta)) + yM; 
      
    %% Store History
    % history of lookahead points
    Quad.lookahead_plot(:,Quad.counter) = [Quad.X_des_GF; Quad.Y_des_GF];
    
    % history of cross track error
    Quad.e_plot(Quad.counter) = e;
    
    Quad.thetaGuess_plot(Quad.counter) = Quad.theta_pos;
    Quad.desired_plot(:,Quad.counter) = [xD; yD];


    
    
end