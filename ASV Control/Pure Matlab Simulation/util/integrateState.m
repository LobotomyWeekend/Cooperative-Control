function [state] = integrateState(ASV, sim, i)
%% Setup
% integration is achieved with TF: 1/s
sys = tf(1,[1,0]);
time = sim.time(1:i);

% extract state history
dStateHist = ASV.dStateHist;

%% Simulate Response to Inputs
if i > 2 % need >2 samples for lsim
    % simulate the following states
    x   = lsim(sys, [dStateHist(1:i).x_dot],  time);
    y   = lsim(sys, [dStateHist(1:i).y_dot],  time);
    yaw = lsim(sys, [dStateHist(1:i).yaw_dot],time);
    u   = lsim(sys, [dStateHist(1:i).u_dot],  time);
    v   = lsim(sys, [dStateHist(1:i).v_dot],  time);
    r   = lsim(sys, [dStateHist(1:i).r_dot],  time);
else
    % required to output ASV.state in all routes
    x = 0;
    y = 0;
    yaw = 0;
    u = 0;
    v = 0;
    r = 0;
end

%% Update Current State
% bias term due to only integrating the state differential
% i.e. need to include initial conditions
state.x     = x(end) + ASV.IC.x;
state.y     = y(end) + ASV.IC.y;
state.z     = 0 + ASV.IC.z;
state.roll  = 0 + ASV.IC.roll;
state.pitch = 0 + ASV.IC.pitch;
state.yaw   = yaw(end) + ASV.IC.yaw;
state.u     = u(end) + ASV.IC.u;
state.v     = v(end) + ASV.IC.v;
state.w     = 0 + ASV.IC.w;
state.p     = 0 + ASV.IC.p;
state.q     = 0 + ASV.IC.q;
state.r     = r(end) + ASV.IC.r;

end