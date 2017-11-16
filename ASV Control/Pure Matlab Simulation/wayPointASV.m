%% wayPointASV
function [ref] = wayPointASV(ASV,ref)
    % gain values
    ku = 0.1;
    ks = 0.01;
    % margin
    % stable if e > 0.5
    e = 0.5;
    
    % extract information
    xD = ref.finish(1,1);
    yD = ref.finish(2,1);
    
    % distance to waypoint
    d = sqrt((ASV.state.x - xD)^2 + (ASV.state.y - yD)^2);
    
    % decide on action
    if (d < e)
        % in region of waypoint, do nothing
        ref.uRef = 0;
    else
        % head to waypoint
        ref.uRef = ku * asin(d/(abs(d) + ks)) * 2/pi;
        ref.yawRef = atan2d((yD - ASV.state.y), (xD - ASV.state.x));
    end  
end