function coordinationUAV(vCorr)
    % establish workspace
    global Quad;
    global Ref;
    
    % find coordination state (gamma)
    Lpos = sqrt((Ref.start(1,1) - Quad.X)^2 + (Ref.start(2,1) - Quad.Y)^2);
    Ltot = sqrt((Ref.start(1,1) - Ref.finish(1,1))^2 + (Ref.start(2,1) - Ref.finish(2,1))^2);
    Quad.gamma = Lpos/Ltot;
    
    % find change in desired position caused by vcorr
    [dxCorr,dyCorr] = posChange(vCorr, Ref.yawD);
    
    % update desired position
    Quad.X_des_GF = Quad.X_des_GF + dxCorr;
    Quad.Y_des_GF = Quad.Y_des_GF + dyCorr;
    
end