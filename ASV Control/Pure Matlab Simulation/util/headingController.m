%% HEADING CONTROLER
% Takes current yaw and yaw rate, attempts to achive yawRef
function [headingCommand, yawIntHold] = headingController(yawRef, ASV, sim)
% Gain Values
Kp = 1500;
Kd = -1000;
Ki = 0.1;

% Get Error Term
err = yawRef - ASV.state.yaw;
% if    (err > 180) 
%      err = err - 360;
% elseif(err < -180) 
%      err = 360 + err;
% end

% Integral error term
yawIntHold = ASV.yawIntHold + err*sim.Ts;

%Provide command
headingCommand = Kp*err + Kd*ASV.state.r + Ki*yawIntHold;

end