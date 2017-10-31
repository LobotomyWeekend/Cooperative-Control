%% Execution
% reference inputs
yawRef = 90;
uRef = 1;
% time inputs
Ts = 0.2;
Tend = 120;
time = 0:Ts:Tend;
% run sim
[stateHist,dStateHist] = innerLoopASV(yawRef,uRef, Ts, Tend);

%% Plots
plot([stateHist.x],[stateHist.y]); % position
% plot(time,[stateHist.yaw]); % yaw over time
% plot(time,[stateHist.u]); % speed over time