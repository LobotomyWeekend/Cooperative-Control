%% VEHICLE DYNAMICS
% Outputs the differential of the state
% Takes inputs as overall thrust, and torque of motors
function [ASV] = vehicleDynamics(ASV, tau_u, tau_r)
%% Assumptions
ASV.dState.z_dot = 0;     % always on water surface
ASV.dState.roll_dot = 0;  % no rolling
ASV.dState.pitch_dot = 0; % no pitching
ASV.dState.w_dot = 0;     % no heave
ASV.dState.p_dot = 0;     % no roll rate
ASV.dState.q_dot = 0;     % no pitch rate

%% Calculate differential of state
% Added Mass
mu =  ASV.properties.m -  ASV.properties.X_u_dot;
mv =  ASV.properties.m -  ASV.properties.Y_v_dot;
mr =  ASV.properties.Iz - ASV.properties.N_r_dot;
muv = mu - mv;

% Hydrodynamic Damping
du = - ASV.properties.X_u - ASV.properties.X_uu * abs(ASV.state.u);
dv = - ASV.properties.Y_v - ASV.properties.Y_vv * abs(ASV.state.v);
dr = - ASV.properties.N_r - ASV.properties.N_rr * abs(ASV.state.r);

% State Derivatives
ASV.dState.u_dot = 1/mu * (tau_u + mv*ASV.state.v*ASV.state.r - du*ASV.state.u);
ASV.dState.v_dot = 1/mv * (- mu*ASV.state.u*ASV.state.r - dv*ASV.state.v);
ASV.dState.r_dot = 1/mr * (tau_r + muv*ASV.state.u*ASV.state.v - dr*ASV.state.r);

% Kinematics
ASV.dState.x_dot   = ASV.state.u*cosd(ASV.state.yaw) - ASV.state.v*sind(ASV.state.yaw);
ASV.dState.y_dot   = ASV.state.u*sind(ASV.state.yaw) + ASV.state.v*cosd(ASV.state.yaw);
ASV.dState.yaw_dot = ASV.state.r;
end
