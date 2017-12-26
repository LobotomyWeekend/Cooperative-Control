function Quad = position_PID(Quad)

    persistent x_error_sum;
    persistent y_error_sum;

    % initialize persistent variables at beginning of simulation
    if Quad.init==0
        x_error_sum = 0;
        y_error_sum = 0;
    end

    %% Define State
    x = Quad.X;
    y = Quad.Y;
    z = Quad.Z;
    phi = Quad.phi;
    theta = Quad.theta;
    psi = Quad.psi;

    % Rotate Desired Position from GF to BF (Z axis rotation only)
    [Quad.X_des,Quad.Y_des,Quad.Z_des] = rotateGFtoBF(Quad.X_des_GF,Quad.Y_des_GF,Quad.Z_des_GF,0*phi,0*theta,psi);

    % Rotate Current Position from GF to BF
    [Quad.X_BF,Quad.Y_BF,Quad.Z_BF] = rotateGFtoBF(x,y,z,phi,theta,psi);

    % Rotate Current Velocity from GF to BF
    [Quad.X_BF_dot,Quad.Y_BF_dot,Quad.Z_BF_dot] = rotateGFtoBF(Quad.X_dot,Quad.Y_dot,Quad.Z_dot,phi,theta,psi);

    %% X Position PID controller 
    x_error = Quad.X_des - Quad.X_BF;
    if(abs(x_error) < Quad.X_KI_lim)
        x_error_sum = x_error_sum + x_error;
    end
    cp = Quad.X_KP*x_error;    %Proportional term
    ci = Quad.X_KI*Quad.Ts*x_error_sum;
    ci = min(Quad.theta_max, max(-Quad.theta_max, ci));    %Saturate ci
    cd = Quad.X_KD*Quad.X_BF_dot;                     %Derivative term
    Quad.theta_des =  - (cp + ci + cd);   %Theta and X inversely related
    Quad.theta_des = min(Quad.theta_max, max(-Quad.theta_max, Quad.theta_des));


    %% Y Position PID controller
    y_error = Quad.Y_des - Quad.Y_BF;
    if(abs(y_error) < Quad.Y_KI_lim)
        y_error_sum = y_error_sum + y_error;
    end
    cp = Quad.Y_KP*y_error;    %Proportional term
    ci = Quad.Y_KI*Quad.Ts*y_error_sum;
    ci = min(Quad.phi_max, max(-Quad.phi_max, ci));    %Saturate ci
    cd = Quad.Y_KD*Quad.Y_BF_dot;                      %Derivative term
    Quad.phi_des = cp + ci + cd; 
    Quad.phi_des = min(Quad.phi_max, max(-Quad.phi_max, Quad.phi_des));

end
















