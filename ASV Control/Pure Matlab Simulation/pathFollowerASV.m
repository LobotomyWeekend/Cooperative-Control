function [ref, ASV] = pathFollowerASV(ASV, ref, sim, i)
%% PATH FOLLOWING controller for ASV
%Input 2 points, generates a path between them and commands the vehicle to
%follow it.

%% Process Path
start = ref.start;
finish = ref.finish;

% gradient [m]
m = (finish(2,1)-start(2,1)) / (finish(1,1)-start(1,1));
if (abs(m) == Inf)
    m = 10000*sign(m);
end

% constant desired yaw [yawD]
yawD = atan2d( (finish(2,1)-start(2,1)) , (finish(1,1)-start(1,1)));

% y intersect [c]
c = start(2,1) - m*start(1,1);

%% Path Error
function u = lineDiff(xL)
    u = (xL - ASV.state.x)^2 + (m*xL + c - ASV.state.y)^2;
end
% find desired position
xD = fminbnd(@lineDiff, ASV.state.x-10, ASV.state.x+10);
yD = m*ASV.state.x + c;
closestPoint = [xD,yD];

% find cross track error
ASV.error.e = sqrt((xD - ASV.state.x)^2 + (yD - ASV.state.y)^2);
if ASV.state.y < (m*ASV.state.x + c)
    ASV.error.e = -ASV.error.e;
end

% save to history
ASV.errorHist(i) = ASV.error;

% find yaw error
ASV.error.yaw = yawD - ASV.state.yaw;

%% Integral
if i == 1
    ASV.error.eInt = 0;
end
ASV.error.eInt = ASV.error.eInt + ASV.error.e*sim.Ts;

%% Provide Yaw Ref
% gain values
K1 =  6.0; %yaw proportional
K2 =  10.0; %cross-track proportional
K4 =  0.0; %integral

% delta term
yawDel = K1*ASV.error.yaw + K2*ASV.error.e/ref.uRef ...
         + K4*ASV.error.eInt;

ref.yawRef = yawDel + yawD;

end

