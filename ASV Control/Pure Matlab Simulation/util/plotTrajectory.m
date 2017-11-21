function plotTrajectory(ASV1, ref1, ASV2, ref2)
%% Look at number of arguments
if nargin < 3
    ASV2 = 0;
    ref2 = 0;
    num = 1;
else 
    num = 2;
end
%% Process desired path
pathType = ref1.pathType;


switch pathType
    case 1 
        %% Straight Line
        % path properties
        [m, c, ~] = processLine(ref1.start,ref1.finish);
  
        % direction +ve or -ve in x
        xs = ref1.start(1,1);
        xf = ref1.finish(1,1);
        if xs > xf
            direc = -0.1;
        else
            direc = 0.1;
        end
        % series of points
        x = xs:direc:xf;% path properties
        y = m*x + c;
        
        %% 2nd Vehicle (optional)
        if num == 2
            [m2,c2, ~] = processLine(ref2.start, ref2.finish);
            xs2 = ref2.start(1,1);
            xf2 = ref2.finish(1,1);
            if xs2 > xf2
                direc = -0.1;
            else
                direc = 0.1;
            end
            x2 = xs2:direc:xf2;% path properties
            y2 = m2*x2 + c2;
        end
    case 2
        %% Arc
        % series of points along the arc in range [0,180] degrees
        [xM, yM, r, ~] = processArc(ref1.start,ref1.finish);
        theta = 0:0.1:180;
        x = r*cosd(theta) + xM;
        y = r*sind(theta) + yM;
        
        %% 2nd Vehicle (Optional)
        if num == 2
            [xM2, yM2, r2, ~] = processArc(ref2.start, ref2.finish);
            x2 = r2*cosd(theta) + xM2;
            y2 = r2*sind(theta) + yM2;
        end
end

%% Plot path taken and desirec
figure('Name', 'Trajectory');
hold on; 
grid on;
axis('equal');

% Plot
plot(x,y,'--r','DisplayName', 'Desired Path ASV1');
plot([ASV1.stateHist.x],[ASV1.stateHist.y], 'DisplayName', 'Taken Path ASV1');
if num ==2
    plot(x2,y2, '--g', 'DisplayName', 'Desired Path ASV2');
    plot([ASV2.stateHist.x], [ASV2.stateHist.y],'k', 'DisplayName', 'Taken Path ASV2');
end

% Centre points
if pathType == 2
    plot(xM,yM, '+k');
    if num == 2
        plot(xM2,yM2,'+k');
    end
end

% Label
legend('show', 'Location', 'best');
xlabel('x (m)');
ylabel('y (m)');

hold off

end