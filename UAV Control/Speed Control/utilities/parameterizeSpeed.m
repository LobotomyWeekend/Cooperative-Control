function UAV = parameterizeSpeed(UAV, ref)
    %% Function to provide x,y,z components of ref velocity to UAV
    UAV.X_dot_GF_des = ref.uRef * cosd(ref.yawRef); % x component
    UAV.Y_dot_GF_des = ref.uRef * sind(ref.yawRef); % y component
    UAV.Z_dot_GF_des = 0; % assumed
end