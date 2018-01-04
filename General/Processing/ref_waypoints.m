function waypoints = ref_waypoints(ref)
%% Function to create a waypoint matrix from a simple ref struct
    % preallocate
    waypoints = zeros(3);
    % point 1
    waypoints(1:2,1) = ref.start;
    waypoints(3,1) = ref.pathType;
    % point 2
    waypoints(1:2,2) = ref.finish;
    waypoints(3,2) = ref.pathType;
    
end