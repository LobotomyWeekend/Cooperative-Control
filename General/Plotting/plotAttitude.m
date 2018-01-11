function plotAttitude(UAV)
    %% Plot attitude refs and repsonses
    % 3 subplots arranged horizontally displaying rotation references and
    % responses in roll, pitch, and yaw against time.
    
    figure('Name','Attitude Controller Tests');
    % roll
    subplot(1,3,1);
    hold on
    grid on
    title('Roll Response');
    xlabel('Time (s)');
    ylabel('Roll (rads)');
    plot(UAV.t_plot, UAV.phi_ref_plot(1:length(UAV.t_plot)), '--');
    plot(UAV.t_plot, UAV.phi_plot(1:length(UAV.t_plot)));
    legend('phi_{ref}','phi');
    hold off
    
    % pitch
    subplot(1,3,2);
    hold on
    grid on
    title('Pitch Response');
    xlabel('Time (s)');
    ylabel('Pitch (rads)');
    plot(UAV.t_plot, UAV.theta_ref_plot(1:length(UAV.t_plot)), '--');
    plot(UAV.t_plot, UAV.theta_plot(1:length(UAV.t_plot)));
    legend('theta_{ref}','theta');
    hold off
    
    % yaw
    subplot(1,3,3);
    hold on
    grid on
    title('Yaw Response');
    xlabel('Time (s)');
    ylabel('Yaw (rads)');
    plot(UAV.t_plot, UAV.psi_ref_plot(1:length(UAV.t_plot)), '--');
    plot(UAV.t_plot, UAV.psi_plot(1:length(UAV.t_plot)));
    legend('psi_{ref}','psi');
    hold off
end