function [ref, V] = componentPath(V, wayPoints, ref)
%% Updates reference values for a component path
% Requires a path divided into sections defined by way points, each time a
% section is completed the function is called and updates the reference 
% to the following section. "wayPoints" should be structured in the following way:
%
%   Section       1     2    ... n
%
%   x           [ x1    x2   ... xn  ]
%   y           [ y1    y2   ... yn  ]
%   pathType    [ 1/2   1/2  ... 1/2 ]

    % Current path section
    V.section;
           
    % Reset coordination state
    V.gamma = 0;

    % Increment to next section
    V.section = V.section + 1;

    % Check if another wayPoint available
    if V.section < length(wayPoints)
        % Reset integral errors
        V.error_crossTrack_int = 0;
        V.yaw_int = 0;

        % Update reference waypoints
        ref.start = wayPoints(1:2, V.section);
        ref.finish = wayPoints(1:2, V.section + 1);
        ref.pathType = wayPoints(3, V.section);
        
        % Initialize PID controllers again
        if V.vehicleType == "UAV"
            V.init = 0;
        end
    else
        disp('Path Completed');
                    
    end % end Sect < length(wayPoints)
        
    
    V.section_init = 0;


end