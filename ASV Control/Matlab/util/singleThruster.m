%% THRUST OF ONE THRUSTER WITH GIVEN RPM
function F = singleThruster(RPM, ASV, j)
             
    % From Mask
    K0 = 7.2115;
    tau = 0.346;
    eta = 4/(1500^2);
    
    % Saturation [-100,100] and round
    if abs(RPM) > 100
        RPM = round(100*sign(RPM));
    else
        RPM = round(RPM);
    end
    
    % Apply inverse Laplace of TF K0/(s + K0)
    time = ASV.time(1 : ASV.counter);
    sys = tf(K0, [1, K0]);
    
    if ASV.counter <= 2
        TF = 0;
        ASV.RPM_plot(:,ASV.counter) = [0;0];
    else
        u    = transpose(ASV.RPM_plot(j, :));
        y    = lsim(sys, u, time);
        TF   = y(end);
    end
        
    % Saturation [-100,100] and round
    if abs(TF) > 100
        TF = round(100*sign(TF));
    else
        TF = round(TF);
    end
    
    % Apply internal gain
    RPM = 45*TF;
    
    % Convert to Force
    F = eta*abs(RPM)*RPM;
end