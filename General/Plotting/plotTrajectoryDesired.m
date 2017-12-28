function plotTrajectoryDesired(ref)
%% Plot the Desired Trajectory from ref struct
% takes the common ref struct and plots the desired trajectory it implies
% in 3D (useful for UAV analysis).

% TODO: plot straight line paths

if ref.pathType == 2 || ref.pathType == 3 % arc path
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
   
elseif ref.pathType == 1 % straight line path
    % get line properties
    [m, c, ~] = processLine(ref.start,ref.finish);
    
    % increasing or decreasing over distance?
    if ref.start(1,1) > ref.finish(1,1)
        inc = -0.1;
    else
        inc = 0.1;
    end
    
    % loop over path length
    j = 1;
    for x = ref.start(1,1) : inc : ref.finish(1,1)
        % x & y coordinates
        ref_plot(:,j) = [x; m*x + c];
        
        j = j + 1;
    end
end

% Output Plot
axis('equal');
grid on;
plot3(ref_plot(1,:), ref_plot(2,:),zeros(1,length(ref_plot)), '--');

end