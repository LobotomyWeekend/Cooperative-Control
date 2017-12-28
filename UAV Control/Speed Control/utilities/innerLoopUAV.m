function UAV = innerLoopUAV(UAV)
    persistent switch_end
    % Control in speed during the mission, point control towards finish
    % point otherwise.
    if ~UAV.complete
        UAV = speed_PID(UAV);
        
        switch_end = 1;
    else
        UAV.complete = 1;
        UAV.X_des_GF = UAV.ref.finish(1,1);
        UAV.Y_des_GF = UAV.ref.finish(2,1);
        if switch_end == 1
            UAV.first_run_position_PID = 1;
            switch_end = 0;
        end
        UAV = position_PID(UAV);
    end
    
    % Common controllers
    UAV = attitude_PID(UAV);
    UAV = rate_PID(UAV);

    % Calculate Desired Motor Speeds
    UAV = quad_motor_speed(UAV);

    % Update Position With The Equations of Motion
    UAV = quad_dynamics_nonlinear(UAV); 

    UAV.init = 1;  %Ends initialization after first simulation iteration 
    
end