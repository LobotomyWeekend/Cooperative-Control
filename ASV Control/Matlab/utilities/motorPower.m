function [ASV] = motorPower(ASV)
%% Provide Motor Power Commands to Thrusters
% take heading and speed commands

% extract the (already squared) speed and heading commands
Cm_sqr = ASV.speedCommand;
Dm_sqr = ASV.headingCommand;

% Value provided to port (Pm) and starboard (Sm) motors, positive moment
% counterclockwise about downwards pointing body fixed z axis (hence the
% change in signs)
ASV.Pm = round(sqrt(abs(Cm_sqr + Dm_sqr)) * sign(Cm_sqr + Dm_sqr));
ASV.Sm = round(sqrt(abs(Cm_sqr - Dm_sqr)) * sign(Cm_sqr - Dm_sqr));

% Plotting RPM values provided
ASV.RPM_plot(:,ASV.counter) = [ASV.Pm; ASV.Sm];
end