    %% Initial Conditions
    function [ASV] = initializeASV(ref, sim)
        % get structure of state
        [ASV.state, ASV.dState] = initializeStuct();
        
        % vehicle properties
        ASV.properties = vehicleProperties();
        
        % vehicle's reference values
        ASV.ref = ref;
        
        % error structure
        ASV.error.e = 0;
        ASV.error.yaw = 0;
        ASV.error.eInt = 0;
        
        % preallocations
        ASV.stateHist = ASV.state;
        ASV.speedHold = 0;
        ASV.yawIntHold = 0;
        ASV.RPM_Hist  = zeros(2,length(sim.time));
        ASV.errorHist = ASV.error;
    end