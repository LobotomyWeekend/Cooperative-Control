%% Coordination Controller for 2 ASVs
% takes the coordination states of two vehicles and calculates the required
% correction velocities, and provides this as a reference
function [uRef1, uRef2] = coordination_2ASV(ASV1, ASV2, ref1, ref2)
    
    %% Setup
    % number of vehicles
    n = 2;
    % extract useful information
    gamma  = [ASV1.coOrd.gamma, ASV2.coOrd.gamma];
    % gain values
    ksync = 1;
    ks = 1;
    
    %% Loop through vehicles to find correction velocity
    % preallocate
    gammaE = zeros(1,n);
    vcorr  = zeros(1,n);
    
    for i = 1:n
        % coordination error
        gammaE(1,i) = gamma(1,i) - 1/n * sum(gamma);
        % correction velocity
        vcorr(1,i) = -ksync * asin(gammaE(1,i) / (abs(gammaE(1,i)) + ks)) * 2/pi;
    end
    
    %% Provide reference values
    uRef1 = ref1.uRefNominal + vcorr(1,1);
    uRef2 = ref2.uRefNominal + vcorr(1,2);
end