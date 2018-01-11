function plotCrossTrackError(V1, V2, V3, V4)
%% Plots cross track error of up to 4 vehicles vs. time
% takes error_crossTrack_plot variable for each and outputs the relevant figure

%% Setup
% Figure properties
figure('Name','Cross Track Error');
hold on;
grid on;

% Number of samples to take
num1 = length(V1.time);
num2 = length(V1.error_crossTrack_plot);
if num1 < num2 
    num = num1;
else
    num = num2;
end


%% Plotting
% Vehicle 1
plot(V1.time(1:num), V1.error_crossTrack_plot(1:num), 'DisplayName', V1.vehicleType);

% Vehicle 2
if nargin >= 2
    plot(V1.time(1:num), V2.error_crossTrack_plot(1:num), 'DisplayName', V2.vehicleType);
end

% Vehicle 3
if nargin >= 3
    plot(V1.time(1:num), V3.error_crossTrack_plot(1:num), 'DisplayName', V3.vehicleType);
end

% Vehicle 4
if nargin >= 4
    plot(V1.time(1:num), V4.error_crossTrack_plot(1:num), 'DisplayName', V4.vehicleType);
end

%% Formatting
xlabel('Time (s)')
ylabel('Cross Track Error (m)');
title('Cross Track Error v. Time');
legend('show');
hold off;

end