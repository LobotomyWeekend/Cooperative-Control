function UAV = endConditionUAV(UAV)
    %% End condition along current path
    % Check if coordination state (i.e. progression along path) has met the
    % completion criteria, and then update the reference.
    if UAV.gamma >= 1 && UAV.counter > 100
        UAV.complete = 1;
    end
   
end