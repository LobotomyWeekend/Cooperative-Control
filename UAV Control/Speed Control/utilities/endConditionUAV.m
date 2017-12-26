function UAV = endConditionUAV(UAV, ref, complete)
    %% End condition along current path
    % Check if coordination state (i.e. progression along path) has met the
    % completion criteria, and then update the reference.
    if UAV.gamma >= complete
        % go to finish
        UAV.X_des_GF = ref.finish(1,1);
        UAV.Y_des_GF = ref.finish(2,1); 
        
        % update error term 
        e = sqrt((UAV.X_des_GF - UAV.X)^2 + (UAV.Y_des_GF - UAV.Y)^2);
        UAV.e_plot(UAV.counter) = e;
        
        % update reference for plot
        UAV.lookahead_plot(1,UAV.counter) = UAV.X_des_GF;
        UAV.lookahead_plot(2,UAV.counter) = UAV.Y_des_GF;

    end
end