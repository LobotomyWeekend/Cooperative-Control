%% EXTRACT USEFUL INFORMATION FROM STATE
% state = [x;y;z;roll;pitch;yaw;dx;dy;dx;droll;dpitch;dyaw]  (12,1)
function [pos, orientation, vel, angularVel] = extractInfoFromState(state)

pos         = state(1  : 3 , 1);
orientation = state(4  : 6 , 1);
vel         = state(7  : 9 , 1);
angularVel  = state(10 : 12, 1);

end
