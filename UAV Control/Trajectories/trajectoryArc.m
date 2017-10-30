%% CALCULATES STRAIGHT LINE TRAJECTORY
% Takes a start and end point
% Creates a 3 times differentiable arc trajectory
% Constant velocity uRef
% point1 = [x1;y1;z1]
% point2 = [x2;y2;z2]

%% EXECUTION
function [pRef] = trajectoryArc(t,uRef,point1,point2)
   % Extract (x,y) of start and end
   x1 = point1(1,1);
   y1 = point1(2,1);
   z1 = point1(3,1);
   
   x2 = point2(1,1);
   y2 = point2(2,1);
   z2 = point2(3,1);
   
   % Average Height, Circle Radius
   h = (z1 + z2) / 2;
   r = sqrt((x2-x1)^2 + (y2-y1)^2)/2;
   
   % Midpoint, Gradient -> Transformation matrix
   xm = (x1 + x2) / 2;
   ym = (y1 + y2) / 2;
   gradAngle = atan((y2-y1)/(x2-x1));
   Trans  = [cos(gradAngle), -sin(gradAngle), 0, xm;
             sin(gradAngle),  cos(gradAngle), 0, ym;
             0             ,  0             , 1, 0 ;
             0             ,  0             , 0, 1 ];
   
   % Angular Velocity
   omega = uRef/r;
   
   % Trajectory
   pRefHold = zeros(4,length(t)); % preallocate
   i = 1;
   for time = t
       theta = pi - omega*time; % position at time
       pos = Trans*[r*cos(theta);r*sin(theta);h;1]; % origin @(0,0) & no inclination
       pRefHold(:,i) = pos; % transformation       
       i = i + 1;
   end
   
   pRef = pRefHold(1:3,:);
end