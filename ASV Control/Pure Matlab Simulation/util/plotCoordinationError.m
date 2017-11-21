%% Analyse and Plot Coordination error over time
% takes the gamma history to calculate the gamma error values as seen by
% the coordination controller
function plotCoordinationError(ASV1,ASV2, sim)
    
    figure ('Name', 'Coordination State');
    
    % Gamma Error Values over Time
    subplot(2,1,1);
    hold on;
    grid on;
    plot(sim.time, [ASV1.coordHist.gammaE], 'DisplayName', 'ASV1');
    plot(sim.time, [ASV2.coordHist.gammaE], 'DisplayName', 'ASV2');
    title('Coordination Error');
    xlabel('time (s)');
    ylabel('gamma ( )');
    legend('show');
    hold off;
    
    % Vcorr Values over time 
    subplot(2,1,2);
    hold on;
    grid on;
    plot(sim.time, [ASV1.coordHist.vcorr], 'DisplayName', 'ASV1');
    plot(sim.time, [ASV2.coordHist.vcorr], 'DisplayName', 'ASV2');
    title('Correction Velocities');
    xlabel('time (s)');
    ylabel('vcorr (m/s)');
    legend('show');
    hold off;    
end