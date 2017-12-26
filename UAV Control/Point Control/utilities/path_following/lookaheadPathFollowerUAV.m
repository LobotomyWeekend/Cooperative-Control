function Quad = lookaheadPathFollowerUAV(Quad, Ref, vcorr)
%% Speed Control
% gain values
K1 = 0; % proportional speed

% current absolute velocity
uReal = sqrt(Quad.X_dot^2 + Quad.Y_dot^2); 
% speed error
uError = uReal - Ref.uRefNominal; 
% new reference velocity
uRef = Ref.uRefNominal - K1*uError + vcorr; 

% keep forwards progression (useful for coordination)
if uRef <= 0
    uRef = 0;
end

%% Path Following
switch Ref.pathType
    case 1 % straight line path
        Quad = straightLineUAV(Ref, Quad, uRef);
    case 2 % clockwise arc
        Quad = arcFollowerUAV(Quad, Ref, uRef, "CW");
    case 3 % counterclockwise arc
        Quad = arcFollowerUAV(Quad, Ref, uRef, "CCW");
    otherwise
        error('Invalid UAV Path Type');
end % end switch pathType
        
    
end