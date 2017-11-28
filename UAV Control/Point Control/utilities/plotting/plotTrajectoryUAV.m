function plotTrajectoryUAV(UAV1,UAV2,UAV3)
    figure('Name', 'Trajectory');
    hold on;
    grid on;
    axis('equal');
    
    % plot vehicle 1
    plot3(UAV1.lookahead_plot(1,:), UAV1.lookahead_plot(2,:),...
        UAV1.Z_ref_plot,'--r','DisplayName','Desired Path UAV 1');
    plot3(UAV1.X_plot, UAV1.Y_plot, UAV1.Z_plot, 'k','DisplayName',...
        'Simulated Path UAV 1');
    
    % check number of inputs
    if nargin == 2
        % plot vehicle 2
        plot3(UAV2.lookahead_plot(1,:), UAV2.lookahead_plot(2,:),...
            UAV2.Z_ref_plot, '--g','DisplayName','Desired Path UAV 2');
        plot3(UAV2.X_plot, UAV2.Y_plot, UAV2.Z_plot, 'k','DisplayName',...
            'Simulated Path UAV 2');
        
    elseif nargin == 3
        % plot vehicle 2
        plot3(UAV2.lookahead_plot(1,:), UAV2.lookahead_plot(2,:),...
            UAV2.Z_ref_plot, '--c','DisplayName','Desired Path UAV 2');
        plot3(UAV2.X_plot, UAV2.Y_plot, UAV2.Z_plot,'k', 'DisplayName',...
            'Simulated Path UAV 2');
        
        % plot vehicle 3
        plot3(UAV3.lookahead_plot(1,:), UAV3.lookahead_plot(2,:),...
            UAV3.Z_ref_plot, '--g','DisplayName','Desired Path UAV 3');
        plot3(UAV3.X_plot, UAV3.Y_plot, UAV3.Z_plot, 'DisplayName',...
            'Simulated Path UAV 3');
    end
            
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
    legend('show');
    hold off;
end