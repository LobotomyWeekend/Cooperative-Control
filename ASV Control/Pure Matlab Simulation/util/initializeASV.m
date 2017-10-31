    %% Initial Conditions
    function [ASV] = initializeASV(ref, sim)
        % get structure
        [ASV.state, ASV.dState] = initializeStuct();
        % initial position
        ASV.state.x = ref.start(1,1);
        ASV.state.y = ref.start(2,1);
        ASV.state.yaw = atan2d(ref.finish(2,1)-ref.start(2,1), ref.finish(1,1)-ref.start(1,1));
        % vehicle properties
        ASV.properties = vehicleProperties();
        % preallocations
        ASV.stateHist = ASV.state;
        ASV.speedHold = 0;
        ASV.RPM_Hist  = zeros(2,length(sim.time));
    end