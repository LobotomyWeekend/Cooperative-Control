% TRAJECTORY PLOTTER
% Takes reference trajectory and position history
% Reference: red dotted line
% History: blue, with initial marker 'x'

function plotTrajectory(pRef,posHist)
    % setup
    figure('Name','Quadrotor Trajectory'); 
    grid on; hold on; view(3);
    
    % plot pRef and posHist
    plot3(pRef(1,:), pRef(2,:), pRef(3,:),'r--'); 
    plot3(posHist(1,:), posHist(2,:), posHist(3,:),'b');
    plot3(posHist(1,1), posHist(2,1), posHist(3,1),'x');
%     plot3(pRef(1,1), pRef(2,1), pRef(3,:),'o');     

    
    % label axes + title
    xlabel('x [m]'); 
    ylabel('y [m]'); 
    zlabel('z [m]');
    legend('Desired Trajectroy','Followed Trajectory');
    title('Quadrotor trajectory');
%     axis equal;
    
    % view angle
    view(15,10);
    hold off
end