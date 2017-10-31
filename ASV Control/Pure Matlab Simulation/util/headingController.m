%% HEADING CONTROLER
% Takes current yaw and yaw rate, attempts to achive yawRef
function headingCommand = headingController(yawRef,state)
% Gain Values
Kp = 45;
Kd = 110;

% Get Required Yaw Terms
yawIn = state.yaw;
yawRateIn = state.r;

% Get Error Term
err = yawRef - yawIn;
if    (err > 180) 
     err = err - 360;
elseif(err < -180) 
     err = 360 + err;
end

%Provide command
headingCommand = Kp*err - Kd*yawRateIn;

end