function ASV = estimateState(ASV)
%% Updates all of the state variables
% a linear state estimation is used, where the time differential is
% multiplied by the length of the time step, and added to the previous
% value of the relevant state variable

%% Setup
% extract time step one for efficiency
Ts = ASV.Ts;

% estimate states in position and orientation
ASV.X     = ASV.X_dot * Ts + ASV.X;
ASV.Y     = ASV.Y_dot * Ts + ASV.Y;
ASV.Z     = ASV.Z_dot * Ts + ASV.Z;
ASV.Roll  = ASV.Roll_dot * Ts + ASV.Roll;
ASV.Pitch = ASV.Pitch_dot * Ts + ASV.Pitch;
ASV.Yaw   = ASV.Yaw_dot * Ts + ASV.Yaw;

% normalize yaw to [0,360]
while ASV.Yaw > 360
    flip = sign(ASV.Yaw);
    ASV.Yaw = ASV.Yaw - flip * 360;
end

% vehicle speeds and rotation rates
ASV.u       = ASV.u_dot * Ts + ASV.u;
ASV.v       = ASV.v_dot * Ts + ASV.v;
ASV.w       = ASV.w_dot * Ts + ASV.w;
ASV.p       = ASV.p_dot * Ts + ASV.p;
ASV.q       = ASV.q_dot * Ts + ASV.q;
ASV.r       = ASV.r_dot * Ts + ASV.r;

%% Plotting variables
% position
ASV.X_plot(ASV.counter) = ASV.X;
ASV.Y_plot(ASV.counter) = ASV.Y;
ASV.Z_plot(ASV.counter) = ASV.Z;

end