%% Find the properties of a line from its start and finish
function [m, c, yawD] = processLine(start,finish)

        % gradient [m]
        m = (finish(2,1)-start(2,1)) / (finish(1,1)-start(1,1));
        % constant desired yaw [yawD]
        yawD = atan2d( (finish(2,1) - start(2,1)) , (finish(1,1) - start(1,1)));
        % y intersect [c]
        c = start(2,1) - m*start(1,1);
        
    end