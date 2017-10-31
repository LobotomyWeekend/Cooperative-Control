%% MOTOR POWER FROM YAW AND SPEED COMMANDS
function [Pm,Sm] = motorPower(headingCommand,speedCommand)

Cm_sqr = speedCommand;
Dm_sqr = headingCommand;

Pm = round(sqrt(abs(Cm_sqr+Dm_sqr))*sign(Cm_sqr+Dm_sqr));
Sm = round(sqrt(abs(Cm_sqr-Dm_sqr))*sign(Cm_sqr-Dm_sqr));
end