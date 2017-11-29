function lawnmowerPath(l,d,start,n)
%% Lawnmower path generator
% l     = track length
% d     = arc diameter
% start = initial conditions;
% n     = number of swaths

%% Find waypoints
points = zeros(2, n*2);
% initial position
points(:,1) = start;
% first swath DOWNWARDS
direc = -1;
% loop over points
for i = 2:2*n
    % check if even
    if mod(i,2) == 0
        % straight line direction "direc" by distance l
        points(:,i) = points(:, i-1) + [0; direc*l];
        
        % define path
        in  = points(2, i-1);
        out = points(2, i);
        % array of points on path
        y = in:0.1*direc:out;
        x = points(2,i)*ones(1,length(y));
        
        direc = direc*(-1);

    else
        % arc right by diameter d
        points(:,i) = points(:,i-1) + [d;0];
        
        % define midpoint
        xm = (points(1,i-1) + points(1,i)) / 2;
        ym = (points(2,i-1) + points(2,i)) / 2;
        % range of angle
        if direc == 1
            theta = 180:360;
        elseif direc == -1
            theta = 180:-1:0;
        end
        % points on path
        x = points(1,i-1):d/180:points(1,i);
        for j = 1:length(x)
            y(1,j) = x(j)*sind(theta(j));
        end
    end  
end


end