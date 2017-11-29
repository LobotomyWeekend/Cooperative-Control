%% MOTOR POWER FROM YAW AND SPEED COMMANDS
function [ASV] = motorPower(ASV)

Cm_sqr = ASV.speedCommand;
Dm_sqr = ASV.headingCommand;

ASV.Pm = round(sqrt(abs(Cm_sqr+Dm_sqr))*sign(Cm_sqr+Dm_sqr));
ASV.Sm = round(sqrt(abs(Cm_sqr-Dm_sqr))*sign(Cm_sqr-Dm_sqr));

% Plotting Variables
ASV.RPM_plot(:,ASV.counter) = [ASV.Pm; ASV.Sm];

end