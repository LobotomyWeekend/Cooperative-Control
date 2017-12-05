function [yawRef, ASV] = pathFollowerASV(ASV, ref)
%% PATH FOLLOWING controller for ASV
% Input 2 points and a path type, generates a path between them and 
% calls the relevant function which commands the vehicle to follow 
% it via a yaw reference (velocity control elsewhere).

switch ref.pathType
    case 1 % straight line
        [yawRef, ASV] = straightLinePath(ASV, ref);
    case 2 % clockwise arc
        [yawRef, ASV] = arcPath(ASV, ref);
    case 3 % counter clockwise arc
        [yawRef, ASV] = arcPath(ASV, ref);
    otherwise
        error('Invalid path type')
end

%% Plotting terms
% cross track error
ASV.error_crossTrack_plot(ASV.counter) = ASV.error_crossTrack;
% coordination state
ASV.gamma_plot(ASV.counter) = ASV.gamma;

end

