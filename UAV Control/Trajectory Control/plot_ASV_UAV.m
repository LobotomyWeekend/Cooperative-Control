% TRAJECTORY PLOTTER
% Takes reference trajectory and position history
% Reference: red dotted line
% History: blue, with initial marker 'x'

function Plot_ASV_UAV(pRef,posHistUAV, posHistASV, tout, start, finish)
    % setup
    figure('Name','Quadrotor Trajectory'); 
    grid on; hold on; view(3);
    
    %% UAV
    % plot pRef and posHist
    plot3(pRef(1,:), pRef(2,:), pRef(3,:),'r--'); 
    plot3(posHistUAV(1,:), posHistUAV(2,:), posHistUAV(3,:),'b');
    plot3(posHistUAV(1,1), posHistUAV(2,1), posHistUAV(3,1),'x');
    plot3(pRef(1,1), pRef(2,1), pRef(3,:),'o');
    
    %% ASV
    [posHist1, posHist2] = arrangeASVData(posHistASV, tout, start, finish);
    % Vehicle 1
    plot3(posHist1(1,:), posHist1(2,:),0, 'r', 'DisplayName','Vehicle 1');
    plot3(mark1(1,:), mark1(2,:),0, 'xr');
    plot3(x(:,1),y(:,1),0,'--', 'DisplayName','Desired Path 1');

    % Vehicle 2
    plot3(posHist2(1,:), posHist2(2,:),0, 'k', 'DisplayName','Vehicle 2');
    plot3(mark2(1,:), mark2(2,:),0, 'xk');
    plot3(x(:,2),y(:,2),'--',0, 'DisplayName','Desired Path 2');

    
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