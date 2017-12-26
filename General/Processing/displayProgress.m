function displayProgress(V)
%% Display the percentage of progress through the simulation over time
% takes the counter and length of the time array to calculate progress as a
% percentage, and displays it in the command window every iteration.

    % clear window
    clc
    % calculate progress
    progress = floor(V.counter / length(V.time) * 100);
    % array to display
    display = [num2str(progress), '% progression'];
    
    % display
    disp(display);

end