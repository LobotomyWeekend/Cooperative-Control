function Quad = coordinationUAV(Quad, vCorr, Ref)
    %% Coordination Process for UAV
    % Finds the coordination state based on path type, this is a normalised
    % value in the range [0,1]. It takes the value of vCorr from the global
    % coordination controller and alters the value of the desired point.
    
    % coordination state for each path type
    switch Ref.pathType
        case 1 % line
            % process line to find yawD
            [~, ~, Ref.yawD] = processLine(Ref.start,Ref.finish);
            
            % length of position vector, and total path length
            Lpos = sqrt((Ref.start(1,1) - Quad.X)^2 + (Ref.start(2,1) - Quad.Y)^2);
            Ltot = sqrt((Ref.start(1,1) - Ref.finish(1,1))^2 + (Ref.start(2,1) - Ref.finish(2,1))^2);
            
            Quad.gamma = Lpos/Ltot;
            
        case 2 % arc
            % current theta_pos as percentage of 180 degree total arc length
            Quad.gamma = (180 - Quad.theta_pos)/180;
            
            % angle of tangent to update desired position
            Ref.yawD = Quad.theta_pos - 90;
            
        otherwise
            error('Invalid Path Type');
    end
       
    % Calculate incrememts
    dxCorr = sqrt(vCorr^2 / ((tand(Ref.yawD))^2 + 1)) * Quad.Ts;
    dyCorr = sqrt(vCorr^2 * (1 - 1/((tand(Ref.yawD))^1 + 1))) * Quad.Ts;
    
    % update desired position
    Quad.X_des_GF = Quad.X_des_GF + dxCorr;
    Quad.Y_des_GF = Quad.Y_des_GF + dyCorr;
    
end