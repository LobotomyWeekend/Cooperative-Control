function [yawRef, ASV] = pathFollowerASV(ASV, ref, sim, i)
%% PATH FOLLOWING controller for ASV
%Input 2 points, generates a path between them and commands the vehicle to
%follow it.

pathType = ASV.ref.pathType;

switch pathType
    case 1
        [yawRef, ASV] = straightLinePath(ASV, ref, sim, i);
    case 2
        [yawRef, ASV] = arcPath(ASV, ref, sim, i);
    otherwise
        error('Invalid path type')
end

end

