function plotErrorUAV(UAV1,ref1, UAV2, ref2)
    % number of vehicles
    n = nargin/2;
    if mod(n,1) ~= 0
        error('Must input pairs of vehicle and ref structs');
    end
    
    % process error
    uReal1 = sqrt(UAV1.X_dot_plot.^2 + UAV1.Y_dot_plot.^2);
    uError1 = uReal1 - ref1.uRefNominal;

    figure('Name', 'Error Terms');

    % cross track error
    subplot(2,n,1);
    title('Cross Track Error (UAV 1)');
    hold on;
    grid on;
    plot(UAV1.t_plot, 10^(3)*UAV1.e_plot(1:length(UAV1.t_plot)));
    xlabel('time (s)');
    ylabel('cross track error (mm)');
    hold off

    % velocity error
    subplot(2,n,2);
    title('Speed Error (UAV 2)');
    hold on;
    grid on;
    plot(UAV1.t_plot, uError1(1:length(UAV1.t_plot)));
    xlabel('time (s)');
    ylabel('velocity error (m/s)');
    hold off;
    
    if n == 2
        % process error
        uReal2 = sqrt(UAV2.X_dot_plot.^2 + UAV2.Y_dot_plot.^2);
        uError2 = uReal2 - ref2.uRefNominal;
        % cross track error
        subplot(2,n,3);
        title('Cross Track Error (UAV 2)');
        hold on;
        grid on;
        plot(UAV2.t_plot, 10^(3)*UAV2.e_plot(1:length(UAV2.t_plot)));
        xlabel('time (s)');
        ylabel('cross track error (mm)');
        hold off

        % velocity error
        subplot(2,n,4);
        title('Speed Error (UAV 2)');
        hold on;
        grid on;
        plot(UAV2.t_plot, uError2(1:length(UAV2.t_plot)));
        xlabel('time (s)');
        ylabel('velocity error (m/s)');
        hold off;
    end
end