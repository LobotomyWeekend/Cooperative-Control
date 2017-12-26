function plotTrajectoryDesired(ref)
%% Plot the Desired Trajectory from ref struct
% takes the common ref struct and plots the desired trajectory it implies
% in 3D (useful for UAV analysis).

% TODO: plot straight line paths

if ref.pathType == 2 % arc path
   % get arc properties
   [xM, yM, r, ~] = processArc(ref.start, ref.finish);
   
   % loop over 180 degrees
   j = 1;
   for theta = 0:0.1:180
       % x & y coordinates
       ref_plot(:,j) = [r*cosd(theta) + xM; r*sind(theta) + yM];
       
       j = j + 1;
   end % end loop over angles
   
else 
    ref_plot = 0;
end

% Output Plot
axis('equal');
grid on;
plot3(ref_plot(1,:), ref_plot(2,:),zeros(1,length(ref_plot)), '--');

end