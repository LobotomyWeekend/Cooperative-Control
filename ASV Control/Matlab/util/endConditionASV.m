function [ref, ASV] = endConditionASV(ASV, ref, complete)
%% End of path controller for ASV
% determines whether near end point of path i.e. gamma = 1, then directs
% the vehicle in its direction until settled.
    
    % gain values
%     ksync = 4.5;
%     ks = 0.01;
    
    % once ASV reaches finish point, turn on end conditions
    if ASV.gamma >= complete && ASV.counter > 100
        ASV.latch = 1;
    end    
       
   
end
