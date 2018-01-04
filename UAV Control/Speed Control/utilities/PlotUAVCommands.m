function PlotUAVCommands(UAV, select)
%% Function to plot reference information of the UAV vs Time

%% FIGURE 1
if select == "All" || select == "Inputs"
    % on four individual subplots within one figure, the control terms will be
    % plotted and clearly labelled.
    % Setup
    figure('Name','UAV Control Inputs');
    % U1
    subplot(2,2,1);
    hold on
    grid on
    plot(UAV.t_plot,UAV.U1_plot);
    hold off
    xlabel('Time (s)');
    ylabel('[ ]');
    title('Thrust Control U_1');
    % U2
    subplot(2,2,2);
    hold on
    grid on
    plot(UAV.t_plot,UAV.U2_plot);
    hold off
    xlabel('Time (s)');
    ylabel('[ ]');
    title('Roll control U_2');
    % U3
    subplot(2,2,3);
    hold on
    grid on
    plot(UAV.t_plot,UAV.U3_plot);
    hold off
    xlabel('Time (s)');
    ylabel('[ ]');
    title('Pitch Control U_3');
    % U4
    subplot(2,2,4);
    hold on
    grid on
    plot(UAV.t_plot,UAV.U4_plot);
    hold off
    xlabel('Time (s)');
    ylabel('[ ]');
    title('Yaw Control U_4');
end


%% FIGURE 2 - Speed Controller
if select == "All" || select == "Speed"
    figure('Name', 'Speed Control')
    % X_dot
    subplot(1,2,1);
    hold on
    grid on
    plot(UAV.t_plot, UAV.X_dot_GF_des_plot,'--b');
    plot(UAV.t_plot, UAV.X_dot_plot(1:length(UAV.t_plot)),'k');
    hold off
    xlabel('Time (s)');
    ylabel('Speed (m/s)');
    legend('Reference','Simulated');
    title('X speed');
    % Y_dot
    subplot(1,2,2);
    hold on
    grid on
    plot(UAV.t_plot, UAV.Y_dot_GF_des_plot,'--b');
    plot(UAV.t_plot, UAV.Y_dot_plot(1:length(UAV.t_plot)),'k');
    hold off
    xlabel('Time (s)');
    ylabel('Speed (m/s)');
    legend('Reference','Simulated');
    title('Y speed');
end

%% FIGURE 3 - Altitude Controller
if select == "All" || select == "Altitude"
    figure('Name','Altitude Controller');
    hold on
    grid on
    plot(UAV.t_plot, UAV.Z_des_GF_plot(1:length(UAV.t_plot)), '--b');
    plot(UAV.t_plot, UAV.Z_plot(1:length(UAV.t_plot)), 'k');
    xlabel('Time (s)');
    ylabel('Altitude (m)');
    legend('Reference','Simulated');
end

%% FIGURE 4 - Attitude Controller
if select == "All" || select == "Attitude"
    figure('Name','Attitude Controller');
    % phi
    subplot(1,3,1);
    hold on
    grid on
    plot(UAV.t_plot, UAV.phi_ref_plot(1:length(UAV.t_plot)), '--b');
    plot(UAV.t_plot, UAV.phi_plot(1:length(UAV.t_plot)),'k');
    hold off
    xlabel('Time (s)');
    ylabel('Roll Angle (rads)');
    legend('Reference','Simulated');
    
    % theta
    subplot(1,3,2);
    hold on
    grid on
    plot(UAV.t_plot, UAV.theta_ref_plot(1:length(UAV.t_plot)), '--b');
    plot(UAV.t_plot, UAV.theta_plot(1:length(UAV.t_plot)),'k');
    hold off
    xlabel('Time (s)');
    ylabel('Pitch Angle (rads)');
    legend('Reference','Simulated');
    
    % psi
    subplot(1,3,3);
    hold on
    grid on
    plot(UAV.t_plot, UAV.psi_ref_plot(1:length(UAV.t_plot)), '--b');
    plot(UAV.t_plot, UAV.psi_plot(1:length(UAV.t_plot)),'k');
    hold off
    xlabel('Time (s)');
    ylabel('Yaw Angle (rads)');
    legend('Reference','Simulated');
end


end