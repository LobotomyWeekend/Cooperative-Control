function pathArray = createPathArray(ref)
    if ref.pathType == 2 || ref.pathType == 3
        %% Arc Path
        % get arc properties
        [xM, yM, r, ~] = processArc(ref.start, ref.finish);
        % choose direction
        if ref.pathType == 2
            loop = 0:0.1:180;
        else
            loop = 180:0.1:360;
        end
        % loop over 180 degrees
        j = 1;
        for theta = loop
            % x & y coordinates
            pathArray(:,j) = [r*cosd(theta) + xM; r*sind(theta) + yM];
            j = j + 1;
        end % end loop over angles       
    elseif ref.pathType == 1
        %% Straight Line Path
        % get line properties
        [m, c, ~] = processLine(ref.start,ref.finish);
        % increasing or decreasing over distance?
        if ref.start(1,1) > ref.finish(1,1)
            inc = -0.1;
        else
            inc = 0.1;
        end
        % Typical Lines
        if abs(m) ~= Inf
            % loop over path length in x
            j = 1;
            for x = ref.start(1,1) : inc : ref.finish(1,1)
                % x & y coordinates
                pathArray(:,j) = [x; m*x + c];
                j = j + 1;
            end   
        % Vertical Lines
        else
            % loop along y
            j = 1;
            for y = ref.start(2,1) : sign(m)/10 : ref.finish(2,1)
                x = ref.start(1,1);
                pathArray(:,j) = [x;y];
                j = j + 1;
            end
        end
    end
end