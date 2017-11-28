function plotErrorUAV(UAV,ref)
    % process error
    uReal = sqrt(UAV.X_dot_plot.^2 + UAV.Y_dot_plot.^2);
    uError = uReal - ref.uRefNominal;

    figure('Name', 'Error Terms');

    % cross track error
    subplot(2,1,1);
    title('Cross Track Error');
    hold on;
    grid on;
    plot(UAV.t_plot, 10^(3)*UAV.e_plot(1:length(UAV.t_plot)));
    xlabel('time (s)');
    ylabel('cross track error (mm)');
    hold off

    % velocity error
    subplot(2,1,2);
    title('Speed Error');
    hold on;
    grid on;
    plot(UAV.t_plot, uError(1:length(UAV.t_plot)));
    xlabel('time (s)');
    ylabel('velocity error (m/s)');
    hold off;
end