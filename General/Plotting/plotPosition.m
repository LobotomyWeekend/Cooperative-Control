function plotPosition(UAV)
    %% Plot attitude refs and repsonses
    % 3 subplots arranged horizontally displaying rotation references and
    % responses in roll, pitch, and yaw against time.
    
    figure('Name','Position Controller Tests');
    % roll
    subplot(1,2,1);
    hold on
    grid on
    title('X^{G} Response');
    xlabel('Time (s)');
    ylabel('X^{G} (m)');
    plot(UAV.t_plot, UAV.X_des_GF_plot(1:length(UAV.t_plot)), '--');
    plot(UAV.t_plot, UAV.X_plot(1:length(UAV.t_plot)));
    legend('X_{ref}','X');
    hold off
    
    % pitch
    subplot(1,2,2);
    hold on
    grid on
    title('Y^{G} Response');
    xlabel('Time (s)');
    ylabel('Y^{G} (m)');
    plot(UAV.t_plot, UAV.Y_des_GF_plot(1:length(UAV.t_plot)), '--');
    plot(UAV.t_plot, UAV.Y_plot(1:length(UAV.t_plot)));
    legend('Y_{ref}','Y');
    hold off
    

end