%% VEHICLE DYNAMICS
% Outputs the differential of the state
% Takes inputs as overall thrust, and torque of motors
function [ASV] = vehicleDynamics(ASV)
%% Assumptions
ASV.Z_dot = 0;      % always on water surface
ASV.Roll_dot = 0;   % no rolling
ASV.Pitch_dot = 0;  % no pitching
ASV.w_dot = 0;     % no heave
ASV.p_dot = 0;     % no roll rate
ASV.q_dot = 0;     % no pitch rate

%% Calculate differential of state
% Added Mass
mu =  ASV.m -  ASV.X_u_dot;
mv =  ASV.m -  ASV.Y_v_dot;
mr =  ASV.Iz - ASV.N_r_dot;
muv = mu - mv;

% Hydrodynamic Damping
du = - ASV.X_u - ASV.X_uu * abs(ASV.u);
dv = - ASV.Y_v - ASV.Y_vv * abs(ASV.v);
dr = - ASV.N_r - ASV.N_rr * abs(ASV.r);

% State Derivatives
ASV.u_dot = 1/mu * (ASV.tau_u + mv * ASV.v * ASV.r - du * ASV.u);
ASV.v_dot = 1/mv * (dv * ASV.v);
ASV.r_dot = 1/mr * (ASV.tau_r + muv * ASV.u * ASV.v - dr * ASV.r);

% Kinematics
ASV.X_dot   = ASV.u * cosd(ASV.Yaw) - ASV.v * sind(ASV.Yaw);
ASV.Y_dot   = ASV.u * sind(ASV.Yaw) + ASV.v * cosd(ASV.Yaw);
ASV.Yaw_dot = ASV.r;

% Counter
ASV.counter = ASV.counter + 1;
end
