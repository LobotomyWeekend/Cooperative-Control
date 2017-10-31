    %% TESTING ASV INNER LOOP
    % need a function which mimics the behaviour of the simulink model
    % i.e. can be placed inside a time loop with ref values changed
    % dynamically.
    
    %% Simulation Inputs
    % time
    sim.Ts   = 0.2;
    sim.Tend = 120;
    sim.time = 0:sim.Ts:sim.Tend;
    
    %% Initialize Vehicles
    % vehicle 1
    ASV1 = initializeASV(ref, sim);
    % waypoints
    ref.start = [0;0];
    ref.finish = [10;10];
    
    i = 1;
    for t = sim.time
        %% Calculate References
        ref.yawRef = atan2d(10,10);
        ref.uRef = 1;
        
        %% Simulate Vehicles
        % ASV 1
        [ASV1] = innerLoopASV(ref, ASV1, sim, i);

        %% Save Data
        % state history
        ASV1.stateHist(i) = ASV1.state;
        i = i + 1;
    
    end
    
    %% Plot Trajectories
    plot([ASV1.stateHist.x],[ASV1.stateHist.y]);