function [posHist1, posHist2] = arrangeASVData(posHist, tout, start, finish)
%% Arranging Data

% find number of vehicles
n = length(posHist.signals.dimensions);

% regularity of marked points 
div = 20;

% number of samples
p = length(tout);
% ... of which are divisible by div
q = floor(tout(end)/div);

% desired line
start  = start(:,:,1);
finish = finish(:,:,1);

for i = 1:n
    x(:,i) = [start(1,i); finish(1,i)];
    y(:,i) = [start(2,i); finish(2,i)];
end

% establish matrices
posHist1 = zeros(2,p);
posHist2 = zeros(2,p);
mark1    = zeros(2,q);
mark2    = zeros(2,q);

% loop over time, return each vehicle's position history
j = 1;
for i = 1:p
    posHist1(:,i) = posHist.signals.values(:,1,i);
    posHist2(:,i) = posHist.signals.values(:,2,i);
    
    % index of tout divisible by div
    if (~mod(tout(i),div))
        mark1(:,j) = posHist1(:, i);
        mark2(:,j) = posHist2(:, i);
        j = j + 1;
    end
end
end