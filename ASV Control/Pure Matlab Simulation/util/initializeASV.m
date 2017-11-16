    %% Initial Conditions
    function [ASV] = initializeASV(ref, sim)
        % get structure of state
        [ASV.state, ASV.dState] = initializeStuct();
        
        % vehicle properties
        ASV.properties = vehicleProperties();
        
        % vehicle's reference values
        ASV.ref = ref;
        
%         % error structure
%         ASV.error.e;
%         ASV.error.yaw;
%         ASV.error.eInt;
                
        % preallocations
        ASV.stateHist = ASV.state;
        ASV.speedHold = 0;
        ASV.yawIntHold = 0;
        ASV.RPM_Hist  = zeros(2,length(sim.time));
%         ASV.errorHist = ASV.error;
    end