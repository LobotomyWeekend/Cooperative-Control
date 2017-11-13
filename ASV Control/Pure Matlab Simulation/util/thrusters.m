%% CALCULATE THRUST AND MOMENT CAUSED BY THRUSTER PAIR
function [tau_u, tau_r, Fs, Fp] = thrusters(RPMs, RPMp, ASV, sim, i)
    % dist between motors
    l = 0.25;
    % thrust from each motor
    Fs = singleThruster(RPMs, ASV, sim, i, 2);
    Fp = singleThruster(RPMp, ASV, sim, i, 1);
    % resultant force and moment
    tau_u = Fs + Fp;
    tau_r = l*(Fp - Fs);
end