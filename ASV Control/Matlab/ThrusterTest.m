%% Thruster Simulator
% Test response of thruster model to stepped inputs, and plot on two
% subplots the percentage of RPM given as a reference and the resultant
% force. This is a lightweight simulator to make adjutments to the thruster
% model by means of a simple simulation. It mimics ASV behaviour as much as
% neccessary. 

% Setup
clear all;
close all;
clc;

% Pseudo ASV
sim.time = 0:0.1:100;
sim.counter = 1;
timeStep = 10;

% Preallocate Matrices
Force_Hist = zeros(1,length(sim.time));
Ref_Hist = zeros(1,length(sim.time));

% RPM variables
RPM = 0; % init
inc = 25; % increment
max = 125; % maximum RPM input
sign = 1; % initially increase

% Constants
eta = 4/(1500^2);

for t = sim.time
    
    % Step RPM input values
    if ~mod(t,timeStep)
        % Update RPM reference
        RPM = RPM + inc * sign;
        
        % Flip sign at RPM_max
        if RPM >= max && sign == 1
            sign = -1;
        end
        
        % Prevent from going below zero
        if RPM <= 0.01 && sign == -1
            sign = 0;
        end
    end
    
    % save RPM hist for TF
    sim.RPM_plot((1:2),sim.counter) = [RPM;0];
    
    % get thruster force
    F = singleThruster(RPM, sim, 1); 
    
    % save history
    Force_Hist(sim.counter) = F; % resultant force
    Ref_Hist(sim.counter) = RPM; % reference
    
    % increment counter
    sim.counter = sim.counter + 1; 
end

%% Plotting
hold on
% Force History
subplot(2,1,1);
plot(sim.time, Force_Hist);
xlabel('time (s)');
ylabel('force (N)');
legend('Resultant Force (N)');

% Rererence
subplot(2,1,2);
plot(sim.time, Ref_Hist, '--');
xlabel('time (s)');
ylabel('(%)');
legend('Reference (%)');
