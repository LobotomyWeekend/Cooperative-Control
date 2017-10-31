%% Initialize state structure
function [state,dState] = initializeStuct()
% Current state
    state.x     = 0;
    state.y     = 0;
    state.z     = 0;
    state.roll  = 0;
    state.pitch = 0;
    state.yaw   = 0;
    state.u     = 0;
    state.v     = 0;
    state.w     = 0;
    state.p     = 0;
    state.q     = 0;
    state.r     = 0;
% Time Differential of State
    dState.x_dot     = 0;
    dState.y_dot     = 0;
    dState.z_dot     = 0;
    dState.roll_dot  = 0;
    dState.pitch_dot = 0;
    dState.yaw_dot   = 0;
    dState.u_dot     = 0;
    dState.v_dot     = 0;
    dState.w_dot     = 0;
    dState.p_dot     = 0;
    dState.q_dot     = 0;
    dState.r_dot     = 0;
end