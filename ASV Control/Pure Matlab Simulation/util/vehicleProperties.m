%% Vehicle Coefficients
% ASV = Autonomous Surface Vehicle
% Struct used for cleaner code and accesibility
function properties = vehicleProperties()
    properties.m       = 17; %kg
    properties.Iz      = 1; %4.14;
    properties.X_u_dot = -20;
    properties.Y_v_dot = -30;%-1.3175;
    properties.N_r_dot = -8.69;% -0.5;
    properties.X_u     = -0.2;
    properties.Y_v     = -50; %-55.117;
    properties.N_r     = -4.14; %-0.1
    properties.X_uu    = -25;
    properties.Y_vv    = -0.01;%-101.2776;
    properties.N_rr    = -6.23; %-21
end