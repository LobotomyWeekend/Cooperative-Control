%% TRAJECTORY TRACKING CONTROLLER FOR UAV %%
clc
close all
%% INPUTS
Ts = 0.01; % step
Tend  = 10; % sim time
uRef = 0.5; % const ref velocity
pathType = 1; % 0 = Line, 1 = Fig 8, 2 = Arc
if (pathType == 0 || pathType == 2)
    % start and end of line
    point1 = [0 ; 0; 1];
    point2 = [10;  10; 1];
end

%% SETUP
% Time vairables
L = 0:Ts:Tend - Ts;
t = 0:Ts:(Tend + 3*Ts);

% Constants
m = 0.2; %mass
g = 9.81; %gravity
omega3 = 0; %const yaw
e1 = [1;0;0]; e2 = [0;1;0]; e3 = [0;0;1]; %unit vectors
K1 = 6*eye(3); K2 = 5*eye(3); K3 = 6*eye(3); %gain values

% Initial Conditions
pos = [0;0;0];
vel = [0;0;0];
T = 0.01;
R = rotx(0)*roty(0)*rotz(0);

% Desired Trajectory
switch pathType
    case 0 % STRAIGHT LINE [point1] -> [point2]
        pRef = trajectoryStraightLine(t,uRef,point1,point2);
    case 1 % FIGURE OF 8      
        pRef = trajectoryFig8(t,uRef);
    case 2
        pRef = trajectoryArc(t,uRef,point1,point2);
    otherwise % ERROR MESSAGE
        error('Error: Trajectory type invalid');
end

% Path Derivatives
dpRef  = diff(pRef, 1, 2)/Ts;
d2pRef = diff(pRef, 2, 2)/Ts^2;
d3pRef = diff(pRef, 3, 2)/Ts^3;

% Preallocate position history
posHist = zeros(3, length(L));     
posHist(:, 1) = pos;

%% EXECUTION
i = 1;
for t = L
    % Position and velocity error
    posErr = pos - pRef(:,i);
    velErr = vel - dpRef(:,i);
    
    % Force in {I}
    F = m*g*e3 - T*R*e3;
    % Desired force condition
    Fd = m*(d2pRef(:,i) - K1*velErr - posErr - K2*(velErr + K1*posErr));
    % Force error
    forceErr = F - Fd;
    
    M = inputMatrix(T);
    
    % Control Input
    mu =  -M\transpose(R)*(-1/m*(velErr + K1*posErr)        ...
                          + m*(d3pRef(:,i)                  ...
                          - (eye(3)-K1^2)*velErr            ...
                          - (K1+K2)*(1/m*forceErr - posErr  ... 
                          - K2*(velErr + K1*posErr)))       ...
                          - K3*forceErr);
    
    u = [mu(1), mu(2), mu(3), omega3];
    
    % Virtual initial condition
    s0 = [pos, vel, R, [T;0;0]];
    
    % Solve dynamics with ode45
    [f, s] = ode45(@RBD, [0 Ts], s0, [], u, m, g);
    
    % Extract Data
    s = transpose(s(end, :));
    p = s(1:3);
    v = s(4:6);
    R = reshape(s(7:15),3,3);
    T = s(16); 
    
    % Save Data
    posHist(:, i+1) = p*1000; 
    i = i + 1;
end

%% Plots
plotUAV(pRef,posHist);
