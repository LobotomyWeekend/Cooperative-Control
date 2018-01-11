function UAV = position_PID(UAV)

    persistent x_error_sum;
    persistent y_error_sum;

    % initialize persistent variables at beginning of simulation
    if UAV.init_position_PID == 0
        x_error_sum = 0;
        y_error_sum = 0;
    end

    %% Define State
    x = UAV.X;
    y = UAV.Y;
    z = UAV.Z;
    phi = UAV.phi;
    theta = UAV.theta;
    psi = UAV.psi;

    % Rotate Desired Position from GF to BF (Z axis rotation only)
    [UAV.X_des,UAV.Y_des,UAV.Z_des] = rotateGFtoBF(UAV.X_des_GF, UAV.Y_des_GF, UAV.Z_des_GF, 0*phi, 0*theta, psi);

    % Rotate Current Position from GF to BF
    [UAV.X_BF,UAV.Y_BF,UAV.Z_BF] = rotateGFtoBF(x,y,z,phi,theta,psi);

    % Rotate Current Velocity from GF to BF
    [UAV.X_BF_dot,UAV.Y_BF_dot,UAV.Z_BF_dot] = rotateGFtoBF(UAV.X_dot,UAV.Y_dot,UAV.Z_dot,phi,theta,psi);

    %% X Position PID controller 
    x_error = UAV.X_des - UAV.X_BF;
    if(abs(x_error) < UAV.X_KI_lim)
        x_error_sum = x_error_sum + x_error;
    end
    cp = UAV.X_KP*x_error;    %Proportional term
    ci = UAV.X_KI*UAV.Ts*x_error_sum;
    ci = min(UAV.theta_max, max(-UAV.theta_max, ci));    %Saturate ci
    cd = UAV.X_KD*UAV.X_BF_dot;                     %Derivative term
    UAV.theta_des =  - (cp + ci + cd);   %Theta and X inversely related
    UAV.theta_des = min(UAV.theta_max, max(-UAV.theta_max, UAV.theta_des));


    %% Y Position PID controller
    y_error = UAV.Y_des - UAV.Y_BF;
    if(abs(y_error) < UAV.Y_KI_lim)
        y_error_sum = y_error_sum + y_error;
    end
    cp = UAV.Y_KP*y_error;    %Proportional term
    ci = UAV.Y_KI*UAV.Ts*y_error_sum;
    ci = min(UAV.phi_max, max(-UAV.phi_max, ci));    %Saturate ci
    cd = UAV.Y_KD*UAV.Y_BF_dot;                      %Derivative term
    UAV.phi_des = cp + ci + cd; 
    UAV.phi_des = min(UAV.phi_max, max(-UAV.phi_max, UAV.phi_des));

end
















