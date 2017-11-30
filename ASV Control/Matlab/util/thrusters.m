%% CALCULATE THRUST AND MOMENT CAUSED BY THRUSTER PAIR
function ASV = thrusters(ASV)
    % dist between motors
    l = 0.25;
    
    % thrust from each motor
    Fs = singleThruster(ASV.Sm, ASV, 2);
    Fp = singleThruster(ASV.Pm, ASV, 1);
    
    % resultant force and moment
    ASV.tau_u = Fs + Fp;
    ASV.tau_r = l*(Fp - Fs);
    
    % Plotting Variables
    ASV.tau_u_plot(ASV.counter) = ASV.tau_u;
    ASV.tau_r_plot(ASV.counter) = ASV.tau_r;
    ASV.Fs_plot(ASV.counter) = Fs;
    ASV.Fp_plot(ASV.counter) = Fp;
end