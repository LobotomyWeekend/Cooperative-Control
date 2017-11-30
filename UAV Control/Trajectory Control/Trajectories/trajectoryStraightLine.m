%% CALCULATES STRAIGHT LINE TRAJECTORY
% Takes a start and end point
% Creates a 3 times differentiable straight line trajectory
% Constant velocity uRef
% point1 = [x1;y1;z1]
% point2 = [x2;y2;z2]

%% EXECUTION
function [pRef] = trajectoryStraightLine(t,uRef,point1,point2)
   % Extract (x,y) of start and end
   x1 = point1(1,1);
   y1 = point1(2,1);
   z1 = point1(3,1);
   
   x2 = point2(1,1);
   y2 = point2(2,1);
   z2 = point2(3,1);
   
   % Prevent singularity
   if (y1 == 0 && y2 == 0)
       y2 = 0.00001;
   elseif (x1 == 0 && x2 ==0)
       x2 = 0.00001;
   end
   
   % Gradient + x-cross + av height
   m = (y2-y1)/(x2-x1);
   theta = atan(m);
   c = y1 - m*x1;
   h = (z1 + z2)/2;
   
   % Velocity in (x,y);
   velX = uRef*cos(theta);
   velY = uRef*sin(theta);
   
   % Four (X,Y) coordinates along the line
   X = linspace(x1,x2,4);
   Y = m*X + c;
   
   % Time between points
   dtX = abs(X(2)-X(1))/velX;
   dtY = abs(Y(2)-Y(1))/velY;
   
   % Trajectory Interpolation
   Tx = zeros(4,4);
   Ty = zeros(4,4);
   for i = 1:4
       Tx(i,:) = [1, dtX*(i-1), (dtX*(i-1))^2, (dtX*(i-1))^3];
       Ty(i,:) = [1, dtY*(i-1), (dtY*(i-1))^2, (dtY*(i-1))^3];
   end
   
   ax = Tx\X';
   ay = Ty\Y';
   
   % Produce tracjectory as matrix
   i = 1;
   pRef = zeros(3,length(t));
   
   for time = t
       pRef(:,i) = [(ax(1) + ax(2)*time + ax(3)*time^2 + ax(4)*time^4);
                  (ay(1) + ay(2)*time + ay(3)*time^2 + ay(4)*time^4);
                  h];
       i = i + 1;
   end
   
end