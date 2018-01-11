%% Coordination Controller for any number of vehicles
% takes the coordination states of up to four vehicles and calculates the required
% correction velocities for each. It is assumed the arguments are entered
% with ascending vehicleID = [1,2,...,4]
function [vcorr] = coordinationMaster(V1, V2, V3, V4)
    %% Setup
    % hold gamma error value
    persistent gammaE_old;
    persistent gammaE_sum;
    if V1.counter == 1
        gammaE_old = zeros(1,nargin);
        gammaE_sum = zeros(1,nargin);
    end
    % time step
    Ts = V1.Ts;
    
    %% Gain Values
    ksync = V1.ref.uRefNominal; % syncronisation gain
    ks = 0.01; % sensitivity gain
    kd = 5; % differential gain
    kd_sat = 0.03; % differential saturation
    ki = 1;
    
    % number of vehicles
    n = nargin;
    
    % preallocate correction velocity matrix
    vcorr  = zeros(1,n);
    % preallocate gamma error matrix
    gammaE = zeros(1,n);
    
    % vehicle types and coordination states of vehicles
    switch nargin
        case 1
            types = [V1.vehicleType];
            gamma = [V1.gamma];
        case 2
            types = [V1.vehicleType, V2.vehicleType];
            gamma = [V1.gamma, V2.gamma];
        case 3 
            types = [V1.vehicleType, V2.vehicleType, V3.vehicleType];
            gamma = [V1.gamma, V2.gamma, V3.gamma];
        case 4 
            types = [V1.vehicleType, V2.vehicleType, V3.vehicleType, V4.vehicleType];
            gamma = [V1.gamma, V2.gamma, V3.gamma, V4.gamma];
    end
    
    
    %% Loop through vehicles to find correction velocity
    for i = 1:n      
        % Coordination error
        gammaE(1,i) = gamma(1,i) - 1/n * sum(gamma);
        % Derivative
        d_gammaE = (gammaE(1,i) - gammaE_old(1,i)) / Ts;
        if abs(d_gammaE) > kd_sat
            d_gammaE = kd_sat * sign(d_gammaE);
        end
        % Integral
        gammaE_sum = gammaE_sum + gammaE;
        gammaE_int = gammaE_sum * Ts;
        % Correction velocity
        vcorr(i) = - ksync * asin(gammaE(1,i) / (abs(gammaE(1,i)) + ks)) * 2/pi - kd * d_gammaE - ki * gammaE_int(1,i);
    end
    
    % Update hold value
    gammaE_old = gammaE;
    
end