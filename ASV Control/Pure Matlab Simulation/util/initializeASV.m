    %% Initial Conditions
    function [ASV] = initializeASV(ref, sim)
        % get structure of state
        [ASV.state, ASV.dState] = initializeStuct();
        
        % vehicle properties
        ASV.properties = vehicleProperties();
        
        % vehicle's reference values
        ASV.ref = ref;
        
        ASV.coOrd.gamma = 0;
        ASV.coOrd.gammaE = 0;
        ASV.coOrd.vcorr = 0;
                
        % preallocations
        ASV.stateHist = ASV.state;
        ASV.speedHold = 0;
        ASV.yawIntHold = 0;
        ASV.RPM_Hist  = zeros(2,length(sim.time));
%         ASV.errorHist = ASV.error;
    end