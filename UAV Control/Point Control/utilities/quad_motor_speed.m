function Quad = quad_motor_speed(Quad)

    % Calculate motor speeds (rad/s)^2
    w1 = Quad.U1/(4*Quad.KT) + Quad.U3/(2*Quad.KT*Quad.l) + Quad.U4/(4*Quad.Kd);
    w2 = Quad.U1/(4*Quad.KT) - Quad.U2/(2*Quad.KT*Quad.l) - Quad.U4/(4*Quad.Kd);
    w3 = Quad.U1/(4*Quad.KT) - Quad.U3/(2*Quad.KT*Quad.l) + Quad.U4/(4*Quad.Kd);
    w4 = Quad.U1/(4*Quad.KT) + Quad.U2/(2*Quad.KT*Quad.l) - Quad.U4/(4*Quad.Kd);

    % Apply realistic motor speed limits
    if w1 > Quad.max_motor_speed^2
        w1 = Quad.max_motor_speed^2;
    end
    if w1 < Quad.min_motor_speed^2
        w1 = Quad.min_motor_speed^2;
    end

    if w2 > Quad.max_motor_speed^2
        w2 = Quad.max_motor_speed^2;
    end
    if w2 < Quad.min_motor_speed^2
        w2 = Quad.min_motor_speed^2;
    end

    if w3 > Quad.max_motor_speed^2
        w3 = Quad.max_motor_speed^2;
    end
    if w3 < Quad.min_motor_speed^2
        w3 = Quad.min_motor_speed^2;
    end

    if w4 > Quad.max_motor_speed^2
        w4 = Quad.max_motor_speed^2;
    end
    if w4 < Quad.min_motor_speed^2
        w4 = Quad.min_motor_speed^2;
    end

    Quad.O1 = sqrt(w1);    % Front M
    Quad.O2 = sqrt(w2);    % Right M
    Quad.O3 = sqrt(w3);    % Rear M
    Quad.O4 = sqrt(w4);    % Left M

    Quad.O1_plot(Quad.counter) = Quad.O1;
    Quad.O2_plot(Quad.counter) = Quad.O2;
    Quad.O3_plot(Quad.counter) = Quad.O3;
    Quad.O4_plot(Quad.counter) = Quad.O4;


    %% Re-compute traditional control inputs

    Quad.U1 = Quad.KT*(Quad.O1^2 + Quad.O2^2 + Quad.O3^2 + Quad.O4^2);
    Quad.U1_plot(Quad.counter) = Quad.U1;

    Quad.U2 = Quad.KT*Quad.l*(Quad.O4^2 - Quad.O2^2);
    Quad.U2_plot(Quad.counter) = Quad.U2;

    Quad.U3 = Quad.KT*Quad.l*(Quad.O1^2 - Quad.O3^2);
    Quad.U3_plot(Quad.counter) = Quad.U3;

    Quad.U4 = Quad.Kd*(Quad.O1^2 + Quad.O3^2 - Quad.O2^2 - Quad.O4^2);
    Quad.U4_plot(Quad.counter) = Quad.U4;

    Quad.O = (Quad.O1 - Quad.O2 + Quad.O3 - Quad.O4);
    Quad.O_plot(Quad.counter) = Quad.O;


end
