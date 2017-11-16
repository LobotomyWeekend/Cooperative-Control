function [state] = estimateState(ASV, sim, i)
%% Setup
Ts = sim.Ts;

% estimate state
state.x     = ASV.dState.x_dot*Ts     + ASV.state.x;
state.y     = ASV.dState.y_dot*Ts     + ASV.state.y;
state.z     = ASV.dState.z_dot*Ts     + ASV.state.z;
state.roll  = ASV.dState.roll_dot*Ts  + ASV.state.roll;
state.pitch = ASV.dState.pitch_dot*Ts + ASV.state.pitch;
state.yaw   = ASV.dState.yaw_dot*Ts   + ASV.state.yaw;
    % normalize yaw to [0,360]
    while state.yaw > 360
        flip = sign(state.yaw);
        state.yaw = state.yaw - flip*360;
    end
state.u     = ASV.dState.u_dot*Ts     + ASV.state.u;
state.v     = ASV.dState.v_dot*Ts     + ASV.state.v;
state.w     = ASV.dState.w_dot*Ts     + ASV.state.w;
state.p     = ASV.dState.p_dot*Ts     + ASV.state.p;
state.q     = ASV.dState.q_dot*Ts     + ASV.state.q;
state.r     = ASV.dState.r_dot*Ts     + ASV.state.r;

% initial conditions
if i == 1
    state = ASV.IC;
end

end