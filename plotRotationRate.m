function plotRotationRate(UAV)
    %% Plot rotation rate refs and repsonses
    % 3 subplots arranged horizontally displaying rotation rate references and
    % responses in [p,q,r] against time.
    
    figure('Name','Rotation Rate Controller Tests');
    % roll
    subplot(1,3,1);
    hold on
    grid on
    title('Roll Rate Response');
    xlabel('Time (s)');
    ylabel('Roll Rate (rads s^{-1})');
    plot(UAV.t_plot, UAV.p_des_plot(1:length(UAV.t_plot)), '--');
    plot(UAV.t_plot, UAV.p_plot(1:length(UAV.t_plot)));
    legend('phi_{ref}','phi');
    hold off
    
    % pitch
    subplot(1,3,2);
    hold on
    grid on
    title('Pitch Rate Response');
    xlabel('Time (s)');
    ylabel('Pitch Rate (rads s^{-1})');
    plot(UAV.t_plot, UAV.q_des_plot(1:length(UAV.t_plot)), '--');
    plot(UAV.t_plot, UAV.q_plot(1:length(UAV.t_plot)));
    legend('q_{ref}','q');
    hold off
    
    % yaw
    subplot(1,3,3);
    hold on
    grid on
    title('Yaw Rate Response');
    xlabel('Time (s)');
    ylabel('Yaw Rate (rads s^{-1})');
    plot(UAV.t_plot, UAV.r_des_plot(1:length(UAV.t_plot)), '--');
    plot(UAV.t_plot, UAV.r_plot(1:length(UAV.t_plot)));
    legend('psi_{ref}','psi');
    hold off
end