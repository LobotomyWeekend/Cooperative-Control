function ref = initialRef(waypoints, nominal_velocity)
    %% Initial reference based on waypoints
    % initial position
    ref.start = waypoints(1:2,1);
    % end of segment 1
    ref.finish = waypoints(1:2,2);
    % path type
    ref.pathType = waypoints(3,1);
    %waypoints
    ref.waypoints = waypoints;
    
    %% Nominal and Initial speed references
    ref.uRefNominal = nominal_velocity;
    ref.uRef = nominal_velocity;
end