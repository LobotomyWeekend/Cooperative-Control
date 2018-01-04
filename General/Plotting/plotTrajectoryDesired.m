function plotTrajectoryDesired(ref, waypoints)
%% Plot the Desired Trajectory from ref struct
% takes the common ref struct and plots the desired trajectory it implies
% in 3D (useful for UAV analysis).

% Check if component path
if nargin > 1
    % store waypoints as
    for i = 1:(length(waypoints) - 2)
        % generate a ref struct of each component line
        ref_waypoint_i.pathType = waypoints(3,i);
        ref_waypoint_i.start = waypoints(1:2,i);
        ref_waypoint_i.finish = waypoints(1:2,i+1);
        % plot each individually
        plotTrajectoryDesired(ref_waypoint_i);
    end
    
    % Plot single component
else
    if ref.pathType == 2 || ref.pathType == 3
        %% Arc Path
        % get arc properties
        [xM, yM, r, ~] = processArc(ref.start, ref.finish);
        
        % choose direction
        if ref.pathType == 2
            loop = 0:0.1:180;
        else
            loop = 180:0.1:360;
        end
        
        % loop over 180 degrees
        j = 1;
        for theta = loop
            % x & y coordinates
            ref_plot(:,j) = [r*cosd(theta) + xM; r*sind(theta) + yM];
            
            j = j + 1;
        end % end loop over angles
        
        
    elseif ref.pathType == 1
        %% Straight Line Path
        % get line properties
        [m, c, ~] = processLine(ref.start,ref.finish);
        
        % increasing or decreasing over distance?
        if ref.start(1,1) > ref.finish(1,1)
            inc = -0.1;
        else
            inc = 0.1;
        end
        
        % Typical Lines
        if abs(m) ~= Inf
            % loop over path length in x
            j = 1;
            for x = ref.start(1,1) : inc : ref.finish(1,1)
                % x & y coordinates
                ref_plot(:,j) = [x; m*x + c];
                j = j + 1;
            end   
        % Vertical Lines
        else
            % loop along y
            j = 1;
            for y = ref.start(2,1) : sign(m)/10 : ref.finish(2,1)
                x = ref.start(1,1);
                ref_plot(:,j) = [x;y];
                j = j + 1;
            end
        end
    end
    
    % Output Plot
    axis('equal');
    grid on;
    plot3(ref_plot(1,:), ref_plot(2,:),zeros(1,length(ref_plot)), '--b');
end

end