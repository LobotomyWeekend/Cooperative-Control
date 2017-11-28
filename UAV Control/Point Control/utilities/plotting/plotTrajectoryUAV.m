function plotTrajectoryUAV(UAV)
    figure('Name', 'Trajectory');
    hold on;
    grid on;
    axis('equal');
    plot3(UAV.lookahead_plot(1,:), UAV.lookahead_plot(2,:), UAV.Z_ref_plot, '--r');
    plot3(UAV.X_plot, UAV.Y_plot, UAV.Z_plot);
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
    hold off;
end