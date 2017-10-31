%% Plot reference values and response
% 1 figure with 2 sub plots, yawRef and uRef
function plotRefValues(ASV, ref, sim)
%% SETUP
    figure('Name', 'Reference Values');
%% yawRef
    subplot(2,1,1);
    hold on;
    grid on;
    plot(sim.time, [ASV.stateHist.yaw]);
    plot(sim.time, [ref.refHist.yawRef], '--');
    legend('Real yaw', 'Yaw Ref', 'Location','best');
    hold off;
%% uRef
    subplot(2,1,2);
    hold on;
    grid on;
    plot(sim.time, [ASV.stateHist.u]);
    plot(sim.time, [ref.refHist.uRef], '--');
    legend('Real u', 'uRef', 'Location','best');
    hold off;
end