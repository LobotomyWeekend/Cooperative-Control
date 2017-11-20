function plotTrajectory(ASV, ref)
%% Process desired path
pathType = ref.pathType;

switch pathType
    case 1 
        %% Straight Line
        % path properties
        [m, c, ~] = processLine(ref.start,ref.finish);
        % direction +ve or -ve in x
        xs = ref.start(1,1);
        xf = ref.finish(1,1);
        if xs > xf
            direc = -0.1;
        else
            direc = 0.1;
        end
        % series of points
        x = xs:direc:xf;% path properties
        y = m*x + c;
    case 2
        %% Arc
        % series of points along the arc in range [0,180] degrees
        [xM, yM, r, ~] = processArc(ref.start,ref.finish);
        theta = 0:0.1:180;
        x = r*cosd(theta) + xM;
        y = r*sind(theta) + yM;
end

%% Plot path taken and desirec
% Begin Figure
figure('Name', 'Trajectory');
hold on; 
% General Settings
grid on;
axis('equal');
% Plot
plot(x,y,'--r'); %desired
plot([ASV.stateHist.x],[ASV.stateHist.y]); %taken
% Label
xlabel('x (m)');
ylabel('y (m)');
hold off; 
end