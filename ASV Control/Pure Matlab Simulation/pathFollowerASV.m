function [yawRef, ASV] = pathFollowerASV(ASV, ref)
%% PATH FOLLOWING controller for ASV
%Input 2 points, generates a path between them and commands the vehicle to
%follow it.

switch ref.pathType
    case 1
        [yawRef, ASV] = straightLinePath(ASV, ref);
    case 2
        [yawRef, ASV] = arcPath(ASV, ref);
    otherwise
        error('Invalid path type')
end

% Plotting terms
ASV.error_crossTrack_plot(ASV.counter) = ASV.error_crossTrack;
ASV.gamma_plot(ASV.counter) = ASV.gamma;

end

