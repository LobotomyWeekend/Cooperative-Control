function [ pd ] = trajectoryFig8(t, V)
%TRAJECTORY Desired trajectory with quadrotor speed V for times t
[~, phi] = ode45(@(t, x) V*sqrt(1 + sin(t)^2), t, t(1));

pd = zeros(3,length(phi));
    for i=1:length(phi)
        pd(:,i) = 3/2*rotx(-pi/4)*rotz(-pi/6)* ...
                  [sin(phi(i))*cos(phi(i))/(sin(phi(i))^2 + 1); ...
                   cos(phi(i))/(sin(phi(i))^2 + 1); ...
                   0] + ...
                  [0;0;-1];
    end
end

