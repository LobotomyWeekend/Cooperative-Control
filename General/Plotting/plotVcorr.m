function plotVcorr(vcorr_hist, time)
    %% Plot vCorr for each of the vehicles over time
    
    % find number of vehicles
    dimensions = size(vcorr_hist);
    no_vehicles = dimensions(1,1);
    
    % setup plot
    figure('Name','Correction Velocities');
    hold on
    grid on
    xlabel ('Time (s)');
    ylabel ('v_{corr}');
    title('Correction Velocities versus Time');
    
    % loop over vehicle and plot
    for i = 1:no_vehicles
        plot(time, vcorr_hist(i,:));
    end
    
    hold off
    
end