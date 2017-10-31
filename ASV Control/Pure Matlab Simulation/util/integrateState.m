function [state] = integrateState(dStateHist, sim, i)

% integration by transfer function 1/s
sys = tf(1,[1,0]);
time = sim.time;

if i > 2
    % linear sim over time
    x   = lsim(sys, [dStateHist(1:i).x_dot],  time);
    y   = lsim(sys, [dStateHist(1:i).y_dot],  time);
    yaw = lsim(sys, [dStateHist(1:i).yaw_dot],time);
    u   = lsim(sys, [dStateHist(1:i).u_dot],  time);
    v   = lsim(sys, [dStateHist(1:i).v_dot],  time);
    r   = lsim(sys, [dStateHist(1:i).r_dot],  time);
else
    x = 0;
    y = 0;
    yaw = 0;
    u = 0;
    v = 0;
    r = 0;
end

% update current state
state.x     = x(end);
state.y     = y(end);
state.z     = 0;
state.roll  = 0;
state.pitch = 0;
state.yaw   = yaw(end);
state.u     = u(end);
state.v     = v(end);
state.w     = 0;
state.p     = 0;
state.q     = 0;
state.r     = r(end);

end