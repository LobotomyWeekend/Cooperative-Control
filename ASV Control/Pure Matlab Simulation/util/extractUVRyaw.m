%% EXTRACT USEFUL INFORMATION FROM STATE
% state = [x;y;z;roll;pitch;yaw;u;v;w;p;q;r]  (12,1)
function [u,v,r,yaw] = extractUVRyaw(state)
u   = state(7  , 1);
v   = state(8  , 1);
r   = state(12 , 1);
yaw = state(6  , 1);
end
