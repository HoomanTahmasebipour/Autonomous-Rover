function [range] = get_ultrasonic(bot_center, bot_rot, sensor_pos, pct_error, fov, maze, block, firstrun, plotmap)
%GET_ULTRASONIC Generates a simulated ultrasonic sensor reading
%   Detailed explanation goes here
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

% Define global plotting variables
global ray_plot
global rayend_plot

% Constants
ray_length = 108; % length of the ray(s) to draw/check for intersection with walls
num_angles = 5;
ray_angles = linspace(-fov/2, fov/2, num_angles); % Determine how the field of view sees the ray

% Set variables if not set by user
if ~exist('firstrun','var')
    firstrun = 1;
end
if ~exist('plotmap','var')
    plotmap = 0;
end

% Determine sensor absolute position
origin = pos_update(bot_center, bot_rot, sensor_pos(1:2));

% Determine sensor absolute orientation
rot = sensor_pos(4) + bot_rot;

% Pre-allocate x, y, and r storage
x_st = zeros(length(ray_length), length(ray_angles));
y_st = zeros(length(ray_length), length(ray_angles));
r_st = ones(length(ray_length), length(ray_angles))*Inf;

% If sensor is lower than the block, append block to maze
if sensor_pos(3) <= abs(block(1,1)-block(2,1))
    maze = [maze;block];
end

for ct = 1:length(ray_length)
    for ct2 = 1:length(ray_angles)
        % Draw line between the sensor and an arbitrary point. This should be
        % longer than the hypotenuse of the maze, so it's not range limited.
        ray = [origin; pos_update(origin, rot+ray_angles(ct2), [ray_length(ct),0])];
        
        % Calculate all the intersection points
        [x,y] = intersections(ray(:,1), ray(:,2), maze(:,1), maze(:,2));
        
        if ~isempty(x)
            % Keep only the first value, it should be the one closest to the origin.
            % Calculate the range from the sensor to the maze
            r = sqrt((ones(size(x))*origin(1) - x).^2 + (ones(size(y))*origin(2) - y).^2);
            
            % Find the smallest value, use that as the range
            [r_st(ct,ct2),i] = min(r);
            
            % Store the minimum x, y, and r value
            x_st(ct,ct2) = x(i);
            y_st(ct,ct2) = y(i);
        end
    end
end


% Determine the minimum r of the values there are
[range_pure,i] = min(r_st);
if length(range_pure)>1
    [range_pure,i2] = min(range_pure);
    ray = [origin; x_st(i(i2),i2), y_st(i(i2),i2)];
else
    ray = [origin; x_st(i), y_st(i)];
end

% Calculate the error and add it to the measurement
range = add_error(range_pure,pct_error);

if plotmap
    if firstrun
        figure(1)
        ray_plot = plot(ray(:,1), ray(:,2), 'r');
        rayend_plot = plot(ray(:,1), ray(:,2), 'ko');
    else
        ray_plot.XData = ray(:,1);
        ray_plot.YData = ray(:,2);
        rayend_plot.XData = ray(:,1);
        rayend_plot.YData = ray(:,2);
    end
end

end

