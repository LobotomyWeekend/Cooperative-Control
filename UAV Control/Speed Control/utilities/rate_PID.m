function Quad = rate_PID(Quad)

    persistent p_error_sum;
    persistent q_error_sum;
    persistent r_error_sum;

    % initialize persistent variables at beginning of simulation
    if Quad.init==0
        p_error_sum = 0;
        q_error_sum = 0;
        r_error_sum = 0;
    end

    p = Quad.p;
    q = Quad.q;
    r = Quad.r;


    %% Angular Rate Controller

    % Roll PID Controller
    p_error = Quad.p_des - p;
    if(abs(p_error) < Quad.p_KI_lim)
        p_error_sum = p_error_sum + p_error;
    end
    cp = Quad.p_KP*p_error;
    ci = Quad.p_KI*Quad.Ts*p_error_sum;
    ci = min(Quad.U2_max, max(Quad.U2_min, ci));
    cd = Quad.p_KD*Quad.p_dot;
    Quad.U2 = cp + ci + cd;
    Quad.U2 = min(Quad.U2_max, max(Quad.U2_min, Quad.U2));

    % Pitch PID Controller
    q_error = Quad.q_des - q;
    if(abs(q_error) < Quad.q_KI_lim)
        q_error_sum = q_error_sum + q_error;
    end
    cp = Quad.q_KP*q_error;
    ci = Quad.q_KI*Quad.Ts*q_error_sum;
    ci = min(Quad.U3_max, max(Quad.U3_min, ci));
    cd = Quad.q_KD*Quad.q_dot;
    Quad.U3 = cp + ci + cd;
    Quad.U3 = min(Quad.U3_max, max(Quad.U3_min, Quad.U3));

    % Yaw PID Controller
    r_error = Quad.r_des - r;
    if(abs(r_error) < Quad.r_KI_lim)
        r_error_sum = r_error_sum + r_error;
    end
    cp = Quad.r_KP*r_error;
    ci = Quad.r_KI*Quad.Ts*r_error_sum;
    ci = min(Quad.U4_max, max(Quad.U4_min, ci));
    cd = Quad.r_KD*Quad.r_dot;
    Quad.U4 = cp + ci + cd;
    Quad.U4 = min(Quad.U4_max, max(Quad.U4_min, Quad.U4));

end