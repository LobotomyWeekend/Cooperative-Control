function plotErrorValues(ASV,sim)
    figure ('Name', 'Error Values');
    subplot(2,1,1);
    hold on; 
    grid on;
    plot(sim.time, [ASV.errorHist.e])
    legend('Cross Track', 'Location','best');
    hold off

    subplot(2,1,2);
    hold on;
    grid on;
    plot(sim.time, [ASV.errorHist.yaw])
    legend('Yaw', 'Location','best');
    hold off
end