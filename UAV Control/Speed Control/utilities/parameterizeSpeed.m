function UAV = parameterizeSpeed(UAV, uRef, yawRef)
    %% Function to provide x,y,z components of ref velocity to UAV
    UAV.X_dot_GF_des = uRef * cosd(yawRef); % x component
    UAV.Y_dot_GF_des = uRef * sind(yawRef); % y component
    UAV.Z_dot_GF_des = 0; % assumed
end