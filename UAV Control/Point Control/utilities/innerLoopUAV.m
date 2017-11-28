function UAV = innerLoopUAV(UAV)
    
    % Implement Controllers
    UAV = position_PID(UAV);
    UAV = attitude_PID(UAV);
    UAV = rate_PID(UAV);

    % Calculate Desired Motor Speeds
    UAV = quad_motor_speed(UAV);

    % Update Position With The Equations of Motion
    UAV = quad_dynamics_nonlinear(UAV); 

    UAV.init = 1;  %Ends initialization after first simulation iteration 
    
end