%% HEADING CONTROLER
% Takes current yaw and yaw rate, attempts to achive yawRef
function ASV = headingController(yawRef, ASV)
% Gain Values
Kp = 1500;
Kd = -1000;
Ki = 0.1;

% Get yaw error 
err = yawRef - ASV.Yaw;

% Integral error term
ASV.yaw_int = ASV.yaw_int + err*ASV.Ts;

% Provide command
ASV.headingCommand = Kp*err + Kd*ASV.r + Ki*ASV.yaw_int;

% Plotting Variable
ASV.headingCommand_plot(ASV.counter) = ASV.headingCommand;
end