function plotTrajectory(V1,V2,V3,V4)
%% Plot trajectories of any combination of vehicles
% Takes up to 4 vehicle objects as arguments, and plots their desired and
% simulated paths.

    %% Setup
    figure('Name', 'Trajectory');
    hold on;
    grid on;
    
    %% Plot Vehicle 1
    if V1.vehicleType == "UAV" % Plot Quadcopter
        
        % Desired Trajectory UAV 1
        plotTrajectoryDesired(V1.ref, V1.vehicleType, V1.ref.waypoints);          
        % Followed Trajectory UAV 1
        plot3(V1.X_plot, V1.Y_plot, V1.Z_plot, 'Color', 'k', 'DisplayName',...
            'Simulated Path UAV 1');
        
    elseif V1.vehicleType == "ASV" % Plot Marine Vehicle
        
       % Desired Trajectory ASV 1
       plotTrajectoryDesired(V1.ref, V1.vehicleType, V1.ref.waypoints);
       % Followed Trajectory ASV 1
       plot3(V1.X_plot, V1.Y_plot, zeros(1,length(V1.X_plot)), 'Color', 'k', ...
           'DisplayName', 'Simulated Path ASV 1');
       
    end
        
    
    %% Plot Vehicle 2
    % check arguments
    if nargin >= 2
        if V2.vehicleType == "UAV" % plot quadcopter
            
            % Desired Trajectory UAV 2
            plotTrajectoryDesired(V2.ref, V2.vehicleType, V2.ref.waypoints);            
            % Followed Trajectory UAV 2
            plot3(V2.X_plot, V2.Y_plot, V2.Z_plot,'DisplayName',...
                'Simulated Path UAV 2');
            
        elseif V2.vehicleType == "ASV" % plot marine vehicle
            
            % Desired Trajectory ASV 2
            plotTrajectoryDesired(V2.ref, V2.vehicleType, V2.ref.waypoints);  
            % Followed Trajectory ASV 2
            plot3(V2.X_plot, V2.Y_plot, zeros(1,length(V2.X_plot)), ...
                'DisplayName', 'Simulated Path ASV 2');
            
        end % end vehicleType
    end %end nargin > = 2
    
    %% Plot Vehicle 3
    % check arguments
    if nargin >= 3        
        if V3.vehicleType == "UAV" % plot quadcopter
            
            % Desired Trajectory UAV 3
            plotTrajectoryDesired(V3.ref, V3.vehicleType, V3.ref.waypoints);              
            % Followed Trajectory UAV 3
            plot3(V3.X_plot, V3.Y_plot, V3.Z_plot, 'DisplayName',...
                'Simulated Path UAV 3');
            
        elseif V3.vehicleType == "ASV" % plot marine vehicle
            
            % Desired Trajectory ASV 3
            plotTrajectoryDesired(V3.ref, V3.vehicleType, V3.ref.waypoints);  
            % Followed Trajectory ASV 3
            plot3(V3.X_plot, V3.Y_plot, zeros(1,length(V3.X_plot)), ...
                'DisplayName', 'Simulated Path ASV 3');
            
        end % end vehicleType
    end % end nargin >= 3

    %% Plot Vehicle 4
    % check arguments
    
    if nargin >= 4        
        if V4.vehicleType == "UAV" % plot quadcopter
            
            % Desired Path UAV 4
            plotTrajectoryDesired(V4.ref, V4.vehicleType, V4.ref.waypoints);        
            % Followed Path UAV 4
            plot3(V4.X_plot, V4.Y_plot, V4.Z_plot, 'DisplayName',...
            'Simulated Path UAV 3');
        
        elseif V4.vehicleType == "ASV" % plot marine vehicle
            
            % Desired Trajectory ASV 4
            plotTrajectoryDesired(V4.ref, V4.vehicleType, V4.ref.waypoints);  
            % Followed Trajectory ASV 4
            plot3(V4.X_plot, V4.Y_plot, zeros(1,length(V4.X_plot)), ...
                'DisplayName', 'Simulated Path ASV 4');
            
        end % end vehicleType
    end % end nargin >= 4
    
    
    %% Formatting
    title('Trajectory');
    xlabel('X^{G} (m)');
    ylabel('Y^{G} (m)');
    zlabel('Z^{G} (m)');
    legend('show');
    hold off;
    
end