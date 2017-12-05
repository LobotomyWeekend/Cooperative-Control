function plotCoordination(V1,V2,V3,V4)
%% Plots coordination states of up to 4 vehicles vs. time
% takes gamma_plot veriable for each and outputs the relevant figure

%% Setup
% Figure properties
figure('Name','Coordination States');
hold on;
grid on;

% Number of samples to take
num = length(V1.time);


%% Plotting
% Vehicle 1
plot(V1.time, V1.gamma_plot(1:num), 'DisplayName', V1.vehicleType);

% Vehicle 2
if nargin >= 2
    plot(V1.time, V2.gamma_plot(1:num), 'DisplayName', V2.vehicleType);
end

% Vehicle 3
if nargin >= 3
    plot(V1.time, V3.gamma_plot(1:num), 'DisplayName', V3.vehicleType);
end

% Vehicle 4
if nargin >= 4
    plot(V1.time, V4.gamma_plot(1:num), 'DisplayName', V4.vehicleType);
end

%% Formatting
xlabel('time (s)')
ylabel('coordination state ( )');
legend('show');
hold off;

end