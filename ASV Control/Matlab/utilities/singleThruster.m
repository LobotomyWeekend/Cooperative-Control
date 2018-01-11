function F = singleThruster(RPM, ASV, j)
%% Function to simulate the behavior of a Medusa Thruster.
% A control value in RPM is provided, ASV is used for time and counter
% calculations, j represents starboard or portside thrusters.
% Model taken from previous work carried out by Jorge Ribeiro at ISR based on
% experimental results.
% A transfer function is applied, this takes the previous up to 100 time steps
% of RPM as inputs.
% Current version ignores time delay (tau) due to previous code errors.s
             
    %% Gain Values
    K0 = 7.2115;
    K1 = 45;
    tau = 0.346;
    eta = 4/(1500^2);
    
    %% Saturation [-100,100] and round
    if abs(RPM) > 100
        RPM = round(100*sign(RPM));
    else
        RPM = round(RPM);
    end
    
    %% Transfer Function
    % Only run transfer function over 100 samples for speed
    % number of samples up to current time
    no_samples = ASV.counter;
    % if less than 100, run over current length
    if ASV.counter < 101
        min = 1;
    else
        min = ASV.counter - 100;
    end
    % generate a time vector
    time = ASV.time(min : no_samples);
    % set the first element in time vector = 0 to avoid errors
    time = time - time(1);
    
    % define the transfer function
    sys = tf(K0, [1, K0]);
  
    % if too few samples for lsim, assume zero output value
    if ASV.counter <= 2
        TF_end = 0;
        ASV.RPM_plot(:,ASV.counter) = [0;0];
    else
        % extract the input vector from RPM_plot
        u    = transpose(ASV.RPM_plot(j, min : no_samples));
        % calculate response
        y    = lsim(sys, u, time);
        % extract response at current time
        TF_end = y(end);
    end
        
    %% Saturation [-100,100] and round
    if abs(TF_end) > 100
        TF_end = round(100*sign(TF_end));
    else
        TF_end = round(TF_end);
    end
    
    %% Output
    % Apply gain
    RPM = K1 * TF_end;
    
    % Convert to Force
    F = eta*abs(RPM)*RPM;
end