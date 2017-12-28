function UAV = innerLoopUAV(UAV)
    % initialization value for end condition
    persistent switch_end
    
    % Control in speed during the mission, point control towards finish
    % point once end criteria met.
    if ~UAV.complete
        switch_end = 1;
        UAV = speed_PID(UAV);
    else
        % initialize position_PID
        if switch_end == 1
            switch_end = 0;
        end
        
        % provide position reference
        UAV.X_des_GF = UAV.ref.finish(1,1);
        UAV.Y_des_GF = UAV.ref.finish(2,1);
        
        UAV = position_PID(UAV);
        
        UAV.init_position_PID = 1;
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