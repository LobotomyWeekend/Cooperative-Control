function [waypoints, ref] = waypointsLawnmower(length_line, diameter_arc, segments)
%% Creates a waypoint matrix for a lawnmower path
% takes straight line length, arc diameter, and number of segments, and
% outputs a standardised waypoint matrix for a lawnmower path, and the
% initial start, finish, and pathType reference values as a struct.

    % Array of path types
    path_types = [1 2 1 3];
    while segments + 1 > length(path_types)
        path_types = [path_types, path_types];
    end

    % Direction of straight line sections
    line_direc = 1;

    % preallocate waypoints matrix
    waypoints = zeros(3, segments + 1);
    % initial conditions assumed at origin
    waypoints(:,1) = [0;0;1];

    % loop through sections
    for i = 2:segments + 1
        switch path_types(i-1)
            case 1 
                % straight line section
                waypoints(1:2,i) = waypoints(1:2, i - 1) + line_direc * [0; length_line]; 
                % switch direction for next straight line
                line_direc = line_direc * (-1);
            case 2
                % clockwise arc
                waypoints(1:2,i) = waypoints(1:2, i-1) + [diameter_arc; 0];
            case 3
                % counter clockwise arc
                waypoints(1:2,i) = waypoints(1:2, i-1) + [diameter_arc; 0];
        end

        % path types starting at waypoints(1:2,i)
        waypoints(3,i) = path_types(i);

    end
    
    % output initial reference
    ref.start = waypoints(1:2,1);
    ref.finish = waypoints(1:2,2);  
    ref.pathType = waypoints(3,1);
    
end