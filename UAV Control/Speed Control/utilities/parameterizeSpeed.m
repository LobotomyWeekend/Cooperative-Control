function UAV = parameterizeSpeed(UAV, yawRef)
    %% Function to provide x,y,z components of ref velocity to UAV
    % prevent coordination controller causing instability
    if UAV.ref.uRef < 0
        UAV.ref.uRef = 0;
    end
    % split into X,Y,Z speed components
    UAV.X_dot_GF_des = UAV.ref.uRef * cosd(yawRef); % x component
    UAV.Y_dot_GF_des = UAV.ref.uRef * sind(yawRef); % y component
    UAV.Z_dot_GF_des = 0; % assumed
end