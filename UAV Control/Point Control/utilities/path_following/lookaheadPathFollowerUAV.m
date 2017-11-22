function lookaheadPathFollowerUAV
    %% Straight Line Path Following
    % establish global UAV variable
    global Quad;
    global Ref;
    
    % find path variables    
    [Ref.m, Ref.c, Ref.yawD] = processLine(Ref.start,Ref.finish);
    
    % find nearest point on line
    xPath = (Quad.X + Quad.Y - Ref.c)/(Ref.m + 1);
    yPath = Ref.m*xPath + Ref.c;
    
    % cross track error
    e = sqrt((xPath - Quad.X)^2 + (yPath - Quad.Y)^2);
    
    % distance to next point in x and y
    [dx,dy] = posChange(Ref.uRefNominal, Ref.yawD);

    %% Update desired position
    Quad.X_des_GF = 1/Ref.m*(yPath + dy - Ref.c);
    Quad.Y_des_GF = Ref.m*(xPath + dx) + Ref.c; 
      
    %% Store History
    % history of lookahead points
    Quad.lookahead_plot(:,Quad.counter) = [Quad.X_des_GF; Quad.Y_des_GF];
    
    % history of cross track error
    Quad.e_plot(Quad.counter) = e;
    
    
end