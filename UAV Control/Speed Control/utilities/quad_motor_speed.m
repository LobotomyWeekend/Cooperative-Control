function UAV = quad_motor_speed(UAV)

    % Calculate motor speeds (rad/s)^2
    w1 = UAV.U1/(4*UAV.KT) + UAV.U3/(2*UAV.KT*UAV.l) + UAV.U4/(4*UAV.Kd);
    w2 = UAV.U1/(4*UAV.KT) - UAV.U2/(2*UAV.KT*UAV.l) - UAV.U4/(4*UAV.Kd);
    w3 = UAV.U1/(4*UAV.KT) - UAV.U3/(2*UAV.KT*UAV.l) + UAV.U4/(4*UAV.Kd);
    w4 = UAV.U1/(4*UAV.KT) + UAV.U2/(2*UAV.KT*UAV.l) - UAV.U4/(4*UAV.Kd);

    % Apply realistic motor speed limits
    if w1 > UAV.max_motor_speed^2
        w1 = UAV.max_motor_speed^2;
    end
    if w1 < UAV.min_motor_speed^2
        w1 = UAV.min_motor_speed^2;
    end

    if w2 > UAV.max_motor_speed^2
        w2 = UAV.max_motor_speed^2;
    end
    if w2 < UAV.min_motor_speed^2
        w2 = UAV.min_motor_speed^2;
    end

    if w3 > UAV.max_motor_speed^2
        w3 = UAV.max_motor_speed^2;
    end
    if w3 < UAV.min_motor_speed^2
        w3 = UAV.min_motor_speed^2;
    end

    if w4 > UAV.max_motor_speed^2
        w4 = UAV.max_motor_speed^2;
    end
    if w4 < UAV.min_motor_speed^2
        w4 = UAV.min_motor_speed^2;
    end

    UAV.O1 = sqrt(w1);    % Front M
    UAV.O2 = sqrt(w2);    % Right M
    UAV.O3 = sqrt(w3);    % Rear M
    UAV.O4 = sqrt(w4);    % Left M

    UAV.O1_plot(UAV.counter) = UAV.O1;
    UAV.O2_plot(UAV.counter) = UAV.O2;
    UAV.O3_plot(UAV.counter) = UAV.O3;
    UAV.O4_plot(UAV.counter) = UAV.O4;


    %% Re-compute traditional control inputs

    UAV.U1 = UAV.KT*(UAV.O1^2 + UAV.O2^2 + UAV.O3^2 + UAV.O4^2);
    UAV.U1_plot(UAV.counter) = UAV.U1;

    UAV.U2 = UAV.KT*UAV.l*(UAV.O4^2 - UAV.O2^2);
    UAV.U2_plot(UAV.counter) = UAV.U2;

    UAV.U3 = UAV.KT*UAV.l*(UAV.O1^2 - UAV.O3^2);
    UAV.U3_plot(UAV.counter) = UAV.U3;

    UAV.U4 = UAV.Kd*(UAV.O1^2 + UAV.O3^2 - UAV.O2^2 - UAV.O4^2);
    UAV.U4_plot(UAV.counter) = UAV.U4;

    UAV.O = (UAV.O1 - UAV.O2 + UAV.O3 - UAV.O4);
    UAV.O_plot(UAV.counter) = UAV.O;


end
