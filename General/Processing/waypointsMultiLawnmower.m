function waypoints = waypointsMultiLawnmower(length_line, diameter_arc_min, offset, segments, no_vehicles)
%% Generates a waypoint matrix for cooperative lawnmower paths
% Accepts n vehicles, and outputs a large [3n, segments + 1] matrix of waypoints
% coordinates and path types. Requires line length, minimum arc diameter (usually
% driven by vehicle properties), offset between paths, and number of
% segments to follow.

% The array structure is the same as in waypointsLawmnower.m, with each
% vehicle stacked un top of each other, i.e.
%
%   VEHICLE 
%      1    [waypoints_1]
%      2    [waypoints_2]
%     ...       ...
%      n    [waypoints_n]

    % Generate array of path types long enough for number of segments
    % normal pattern: line up, arc (CW), line down, arc (CCW)
    path_types = [1 2 1 3];
    while segments + 1 > length(path_types)
        path_types = [path_types, path_types];
    end
    
    % Direction of straight line sections (switches each time)
    line_direc = ones(1, no_vehicles);
    
    % preallocate waypoints matrix
    waypoints = zeros(3*no_vehicles, segments + 1);
    
    % find initial positions
    for i = 0 : no_vehicles - 1
        waypoints(1 + (3*i) : 3 + (3*i), 1) = [offset* i ; 0 ; 1];
    end
    
    % loop over vehicles
    for j = 1 : no_vehicles
        % loop over segments
        for i = 2 : segments + 1
            
            % insert waypoint coordinates based on path type
            switch path_types(i-1)
                case 1 
                    % straight line section
                    waypoints(1 + (3 * (j - 1)) : 2 + (3 * (j - 1)) , i) = ...
                        waypoints(1 + (3 * (j - 1)) : 2 + (3 * (j - 1)), i - 1) + line_direc(j) * [0; length_line]; 
                    % switch direction for next straight line
                    line_direc(j) = line_direc(j) * (-1);
                case 2
                    % find diameter
                    diameter_arc = diameter_arc_min + (no_vehicles - j) * 2 * offset;
                    % clockwise arc
                    waypoints(1 + (3 * (j - 1)) : 2 + (3 * (j - 1)) , i) = ...
                        waypoints(1 + (3 * (j - 1)) : 2 + (3 * (j - 1)) , i-1) + [diameter_arc; 0];
                case 3
                    % find diameter
                    diameter_arc = diameter_arc_min + (j - 1) * 2 * offset;
                    % counter clockwise arc
                    waypoints(1 + (3 * (j - 1)) : 2 + (3 * (j - 1)) , i) = ...
                        waypoints(1 + (3 * (j - 1)) : 2 + (3 * (j - 1)) , i-1) + [diameter_arc; 0];
            
            end % switch path type
            
            % insert path type in 3rd row
            waypoints (3 + (3 * (j - 1)), i) = path_types(i);
            
        end % loop through sections
    end % loop through vehicles

    % note:
        % waypoints_1 = waypoints(1:3, :);
        % waypoints_2 = waypoints(4:6, :);
    % etc.

end
