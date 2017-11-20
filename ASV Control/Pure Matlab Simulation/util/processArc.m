%% Find the properties of the arc from its start and finish
function [xM, yM, r, m] = processArc(start,finish)
    % find midpoint
    xM = (start(1,1) + finish(1,1))/2;
    yM = (start(2,1) + finish(2,1))/2;

    % find radius
    r  = sqrt((finish(1,1) - xM)^2 + (finish(2,1) - yM)^2);

    % find gradient
    m  = (finish(2,1) - start(2,1)) / (finish(1,1) - start(1,1));
end