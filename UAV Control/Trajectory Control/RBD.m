% QUADROTOR Rigid Body Dynamics
% INPUTS
% s     = [x;y;z;vx;vy;vz;reshape(R);T]
% u     = [thrustRateIn; omega1; omega2; omega3]
% R     = Rotation matrix {I} -> {B}
% m     = Constant mass
% g     = Constant gravitaional acceleration
function ds = RBD(t, s, u, m, g)
    % Extract info from state s
    pos = s(1:3);
    vel = s(4:6);
    R = reshape(s(7:15),3,3);
    T = s(16);
    
    % Extract info from input u
    thrustRateIn = u(1,1);
    omegaIn = u(1,2:4);
    
    % Rigid Body Dynamics
    dp = vel;
    dv = m\(-T*R*[0;0;1] + m*g*[0;0;1]);
    dR = R*skew(omegaIn);
    dT = thrustRateIn;
      
    % Reshape for output
    dT = [dT; 0; 0];
    
    % Derivitive of state
    ds = reshape([dp, dv, dR, dT], 18, 1);
end