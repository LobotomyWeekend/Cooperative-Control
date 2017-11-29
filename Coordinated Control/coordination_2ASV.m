%% Coordination Controller for 2 ASVs
% takes the coordination states of two vehicles and calculates the required
% correction velocities, and provides this as a reference
<<<<<<< Updated upstream
function [uRef1, uRef2, ASV1, ASV2] = coordination_2ASV(ASV1, ASV2, ref1, ref2)
=======
function [vCorr1, vCorr2] = coordination_2UAV(UAV1, UAV2)
>>>>>>> Stashed changes
    
    %% Setup
    % number of vehicles
    n = 2;
    % extract useful information
    gamma  = [ASV1.coOrd.gamma, ASV2.coOrd.gamma];
    % gain values
    ksync = 7.5;
    ks = 0.01;
    
    %% Loop through vehicles to find correction velocity
    % preallocate
    gammaE = zeros(1,n);
    vcorr  = zeros(1,n);
    
    for i = 1:n
        % coordination error
        gammaE(1,i) = gamma(1,i) - 1/n * sum(gamma);
        % correction velocity
        vcorr(1,i) = - ksync * asin(gammaE(1,i) / (abs(gammaE(1,i)) + ks)) * 2/pi;
    end
    
    %% Save to vehicle structures
    ASV1.coOrd.gammaE = gammaE(1,1);
    ASV2.coOrd.gammaE = gammaE(1,2);
    ASV1.coOrd.vcorr  =  vcorr(1,1);
    ASV2.coOrd.vcorr  =  vcorr(1,2);
    
    %% Provide reference values
    uRef1 = ref1.uRefNominal + vcorr(1,1);
    uRef2 = ref2.uRefNominal + vcorr(1,2);
end