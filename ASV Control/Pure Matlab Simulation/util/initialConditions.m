    %% Initial Conditions
    function [IC, state] = initialConditions(ref, yawInit)
        
        % initial conditions
        IC.x = ref.start(1,1);
        IC.y = ref.start(2,1);
        IC.z = 0;
        IC.yaw = yawInit;
        IC.pitch = 0;
        IC.roll = 0;
        IC.u = 0;
        IC.v = 0;
        IC.w = 0;
        IC.p = 0;
        IC.q = 0;
        IC.r = 0;
        
        state = IC;
        
    end