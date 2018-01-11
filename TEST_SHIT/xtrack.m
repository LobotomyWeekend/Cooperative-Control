%% Define a path
% start point
x1 = 0;
y1 = 0;
start_pt = [x1;y1];
% end point
x2 = 10;
y2 = 10;
end_pt = [x2;y2];
% find desired yaw angle
yawD = atan2d((y2-y1),(x2-x1));

%% Vehicle position
xv = 1.1;
yv = 1.5;
pos = [xv;yv];

%% Transform to have path on x axis
% transformation matrix
R = [cosd(-yawD),-sind(-yawD);sind(-yawD),cosd(-yawD)];

pos_new = R*pos;

e = pos_new(2);