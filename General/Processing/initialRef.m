function ref = initialRef(waypoints)
    %% Calculates the initial reference based on waypoints
    % initial position
    ref.start = waypoints(1:2,1);
    % end of segment 1
    ref.finish = waypoints(1:2,2);
    % path type
    ref.pathType = waypoints(3,1);
end