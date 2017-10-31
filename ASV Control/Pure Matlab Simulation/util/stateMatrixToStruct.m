function [state] = stateMatrixToStruct(stateMat)
    state.x     = stateMat(1,1);
    state.y     = stateMat(2,1);
    state.z     = stateMat(3,1);
    state.roll  = stateMat(4,1);
    state.pitch = stateMat(5,1);
    state.yaw   = stateMat(6,1);
    state.u     = stateMat(7,1);
    state.v     = stateMat(8,1);
    state.w     = stateMat(9,1);
    state.p     = stateMat(10,1);
    state.q     = stateMat(11,1);
    state.r     = stateMat(12,1);
end