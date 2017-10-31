    %% Initial Conditions
    function [ASV] = initializeASV(ref, sim)
        % get structure of state
        [ASV.state, ASV.dState] = initializeStuct();
        
        % error structure
        ASV.error.e = 0;
        ASV.error.yaw = 0;
        ASV.error.eInt = 0;
        
        % initial conditions
        ASV.state.x = ref.start(1,1);
        ASV.state.y = ref.start(2,1);
        ASV.state.yaw = atan2d(ref.finish(2,1)-ref.start(2,1), ref.finish(1,1)-ref.start(1,1));
        ASV.IC = ASV.state; % put in struct section
        
        % vehicle properties
        ASV.properties = vehicleProperties();
        
        % vehicle's reference values
        ASV.ref = ref;
        
        % preallocations
        ASV.stateHist = ASV.state;
        ASV.speedHold = 0;
        ASV.RPM_Hist  = zeros(2,length(sim.time));
        ASV.errorHist = ASV.error;
    end