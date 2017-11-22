%% Incremental change in position along a path
% Takes an absolute velocity and yaw, and caluclates the distance to the
% next point along the path in x and y

function [dx,dy] = posChange(uRef, yawD)
    
    % Setup workspace
    global Quad
    
    % Calculate incrememts
    dx = sqrt(uRef^2 / ((tand(yawD))^2 + 1)) * Quad.Ts;
    dy = sqrt(uRef^2 * (1 - 1/((tand(yawD))^1 + 1))) * Quad.Ts;

end