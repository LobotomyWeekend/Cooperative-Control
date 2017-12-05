function Quad = straightLineUAV(Ref, Quad, uRef)
%% Straight Line Path Following
    % find path variables    
    [Ref.m, Ref.c, Ref.yawD] = processLine(Ref.start,Ref.finish);
    Quad.yaw_desired = Ref.yawD;

    if abs(Ref.m) ~= Inf % standard case
        % find nearest point on line
        xPath = (Quad.X + Quad.Y - Ref.c)/(Ref.m + 1);
        yPath = Ref.m*xPath + Ref.c;

        % Calculate incrememts
        dx = sqrt(uRef^2 / ((tand(Ref.yawD))^2 + 1)) * Quad.Ts;
        dy = sqrt(uRef^2 * (1 - 1/((tand(Ref.yawD))^2 + 1))) * Quad.Ts;

        % update desired position
        Quad.X_des_GF = 1/Ref.m*(yPath + dy - Ref.c);
        Quad.Y_des_GF = Ref.m*(xPath + dx) + Ref.c; 

    elseif abs(Ref.m) == Inf % vertical line
        % find nearest point on line
        xPath = Ref.start(1,1);
        yPath = Quad.Y;

        % Calculate increments
        dx = 0;
        dy = uRef * Quad.Ts * sign(Ref.m);

        % update desired position
        Quad.X_des_GF = xPath + dx;
        Quad.Y_des_GF = yPath + dy; 
    end

    % cross track error
    e = sqrt((xPath - Quad.X)^2 + (yPath - Quad.Y)^2);



    %% Coordination State
    % length of position vector, and total path length
    Lpos = sqrt((Ref.start(1,1) - Quad.X)^2 + (Ref.start(2,1) - Quad.Y)^2);
    Ltot = sqrt((Ref.start(1,1) - Ref.finish(1,1))^2 + (Ref.start(2,1) - Ref.finish(2,1))^2);

    Quad.gamma = Lpos/Ltot;
    
    %% Store History
    % history of lookahead points
    Quad.lookahead_plot(:,Quad.counter) = [Quad.X_des_GF; Quad.Y_des_GF];
    
    % history of cross track error
    Quad.error_crossTrack_plot(Quad.counter) = e;


end