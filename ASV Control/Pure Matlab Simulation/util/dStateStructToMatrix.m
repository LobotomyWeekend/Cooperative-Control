function dStateMat = dStateStructToMatrix(dState)

dStateMat = [dState.x_dot;
             dState.y_dot;
             dState.z_dot;
             dState.roll_dot;
             dState.pitch_dot;
             dState.yaw_dot;
             dState.u_dot;
             dState.v_dot;
             dState.w_dot;
             dState.p_dot;
             dState.q_dot;
             dState.r_dot];
         
end