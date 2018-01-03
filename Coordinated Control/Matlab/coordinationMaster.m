%% Coordination Controller for any number of vehicles
% takes the coordination states of up to four vehicles and calculates the required
% correction velocities for each. It is assumed the arguments are entered
% with ascending vehicleID = [1,2,...,4]
function [vcorr] = coordinationMaster(V1, V2, V3, V4)
    
    %% Setup    
    % gain values
    ksync = 1;
    ks = 0.01;
    
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
            types = [V1.vehicletype, V2.vehicleType, V3.vehicleType, V4.vehicleType];
            gamma = [V1.gamma, V2.gamma, V3.gamma, V4.gamma];
    end
    
    
    %% Loop through vehicles to find correction velocity
    for i = 1:n      
        % Coordination error
        gammaE(1,i) = gamma(1,i) - 1/n * sum(gamma);
        % Correction velocity
        vcorr(i) = - ksync * asin(gammaE(1,i) / (abs(gammaE(1,i)) + ks)) * 2/pi;
    end
    
end