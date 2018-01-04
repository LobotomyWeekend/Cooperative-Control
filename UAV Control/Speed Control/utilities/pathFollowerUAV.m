function [UAV] = pathFollowerUAV(UAV, ref)
%% PATH FOLLOWING controller for UAV
% Input 2 points and a path type, generates a path between them and 
% calls the relevant function which commands the vehicle to follow 
% it via a yaw reference (velocity control elsewhere).

switch ref.pathType
    case 1 % straight line
        [heading_ref, UAV] = straightLinePathUAV(UAV);
    case 2 % clockwise arc
        [heading_ref, UAV] = arcPathUAV(UAV);
    case 3 % counter clockwise arc
        [heading_ref, UAV] = arcPathUAV(UAV);
    otherwise
        error('Invalid path type')
end

%% Update Reference
UAV = parameterizeSpeed(UAV, heading_ref);

%% Plotting terms
% yaw ref hist
UAV.heading_ref_plot(UAV.counter) = heading_ref;
% cross track error
UAV.error_crossTrack_plot(UAV.counter) = UAV.error_crossTrack;
% coordination state
UAV.gamma_plot(UAV.counter) = UAV.gamma;

UAV.section_init = 1;

end

