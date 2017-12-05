function plotCrossTrackError(V1, V2, V3, V4)
%% Plots cross track error of up to 4 vehicles vs. time
% takes error_crossTrack_plot variable for each and outputs the relevant figure

%% Setup
% Figure properties
figure('Name','Cross Track Error');
hold on;
grid on;

% Number of samples to take
num = length(V1.time);


%% Plotting
% Vehicle 1
plot(V1.time, V1.error_crossTrack_plot(1:num), 'DisplayName', V1.vehicleType);

% Vehicle 2
if nargin >= 2
    plot(V1.time, V2.error_crossTrack_plot(1:num), 'DisplayName', V2.vehicleType);
end

% Vehicle 3
if nargin >= 3
    plot(V1.time, V3.error_crossTrack_plot(1:num), 'DisplayName', V3.vehicleType);
end

% Vehicle 4
if nargin >= 4
    plot(V1.time, V4.error_crossTrack_plot(1:num), 'DisplayName', V4.vehicleType);
end

%% Formatting
xlabel('time (s)')
ylabel('cross track error (m)');
legend('show');
hold off;

end