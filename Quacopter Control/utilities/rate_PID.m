function UAV = rate_PID(UAV)

    persistent p_error_sum;
    persistent q_error_sum;
    persistent r_error_sum;

    % initialize persistent variables at beginning of simulation
    if UAV.init==0
        p_error_sum = 0;
        q_error_sum = 0;
        r_error_sum = 0;
    end

    p = UAV.p;
    q = UAV.q;
    r = UAV.r;


    %% Angular Rate Controller

    % Roll PID Controller
    p_error = UAV.p_des - p;
    if(abs(p_error) < UAV.p_KI_lim)
        p_error_sum = p_error_sum + p_error;
    end
    cp = UAV.p_KP*p_error;
    ci = UAV.p_KI*UAV.Ts*p_error_sum;
    ci = min(UAV.U2_max, max(UAV.U2_min, ci));
    cd = UAV.p_KD*UAV.p_dot;
    UAV.U2 = cp + ci + cd;
    UAV.U2 = min(UAV.U2_max, max(UAV.U2_min, UAV.U2));

    % Pitch PID Controller
    q_error = UAV.q_des - q;
    if(abs(q_error) < UAV.q_KI_lim)
        q_error_sum = q_error_sum + q_error;
    end
    cp = UAV.q_KP*q_error;
    ci = UAV.q_KI*UAV.Ts*q_error_sum;
    ci = min(UAV.U3_max, max(UAV.U3_min, ci));
    cd = UAV.q_KD*UAV.q_dot;
    UAV.U3 = cp + ci + cd;
    UAV.U3 = min(UAV.U3_max, max(UAV.U3_min, UAV.U3));

    % Yaw PID Controller
    r_error = UAV.r_des - r;
    if(abs(r_error) < UAV.r_KI_lim)
        r_error_sum = r_error_sum + r_error;
    end
    cp = UAV.r_KP*r_error;
    ci = UAV.r_KI*UAV.Ts*r_error_sum;
    ci = min(UAV.U4_max, max(UAV.U4_min, ci));
    cd = UAV.r_KD*UAV.r_dot;
    UAV.U4 = cp + ci + cd;
    UAV.U4 = min(UAV.U4_max, max(UAV.U4_min, UAV.U4));

end