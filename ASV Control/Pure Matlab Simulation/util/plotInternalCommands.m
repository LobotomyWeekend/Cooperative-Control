%% Plot Internal Commands vs time
function plotInternalCommands(ASV,sim)

    figure('Name', 'Commands');
    
    % Speed and Heading Commands
    subplot(4,1,1);
    hold on;
    grid on;
    plot(sim.time, [ASV.cmdHist.speedCommand]);
    plot(sim.time, [ASV.cmdHist.headingCommand]);
    legend('Speed Command', 'Heading Command', 'Location','best');
    hold off;
    
    % Force and Moments
    subplot(4,1,2);
    hold on;
    grid on;
    plot(sim.time, [ASV.cmdHist.tau_r]);
    plot(sim.time, [ASV.cmdHist.tau_u]);
    legend('tau_r', 'tau_u', 'Location','best');
    hold off;
    
    % Resultant RPM (port and starboard)
    subplot(4,1,3);
    hold on;
    grid on;
    plot(sim.time, [ASV.cmdHist.RPMs]);
    plot(sim.time, [ASV.cmdHist.RPMp]);
    legend('RPMs', 'RPMp', 'Location','best');
    hold off;
    
    % Resultant Force from each thruster
    subplot(4,1,4);
    hold on;
    grid on;
    plot(sim.time, [ASV.cmdHist.Fs]);
    plot(sim.time, [ASV.cmdHist.Fp]);
    legend('Fs', 'Fp', 'Location','best');
    hold off;
end