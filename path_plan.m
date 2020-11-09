function [movement] = path_plan(cmd_id, cmd_data, drive, num_segments)
%PATH_PLAN Plans a path for a robot to follow
%   This function takes in data about the robot and maze and outputs a set
%   of movements for the robot to follow. An ideal final position is
%   generated (relative to the start position), then that is broken up into
%   segments. Each segment consists of both a movement and rotation. A
%   different function will convert the relative motion segments into
%   the global coordinate system.
%   
%   [movement] is a matrix indicating how the robot should move on its path
%   to properly simulate the command given to it by the user, including
%   errors. Each row indicated a movement action. Another function reads in
%   this data, using it to move the robot in the simulated maze.
%   
%   [id number, rotation, x distance, y distance]
%   
%   id number - the number of the movement
%   rotation - rotation in degrees
%   x distance - the x distance to move in inches
%   y distance - the y distance to move in inches
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

% Preallocation of variables
x = 0;
y = 0;
r = 0;

% Check for valid data
if isnan(cmd_data)
    error(strcat('Command Data "', sprintf('%.2f',cmd_data),'" not valid. Ensure format is as follows - "a1-##".'))
end
    
% Determine the new target position/rotation
switch cmd_id
    case 'up'
        y = cmd_data;
        x_err = [drive.bias_x(1) drive.err_x(1)];
        y_err = [drive.bias_y(1) drive.err_y(1)];
        r_err = [drive.bias_r(1) drive.err_r(1)];
    case 'down'
        y = -cmd_data;
        x_err = [drive.bias_x(2) drive.err_x(2)];
        y_err = [drive.bias_y(2) drive.err_y(2)];
        r_err = [drive.bias_r(2) drive.err_r(2)];
    case 'left'
        x = -cmd_data;
        x_err = [drive.bias_x(3) drive.err_x(3)];
        y_err = [drive.bias_y(3) drive.err_y(3)];
        r_err = [drive.bias_r(3) drive.err_r(3)];
    case 'right'
        x = cmd_data;
        x_err = [drive.bias_x(4) drive.err_x(4)];
        y_err = [drive.bias_y(4) drive.err_y(4)];
        r_err = [drive.bias_r(4) drive.err_r(4)];
    case 'rot'
        r = cmd_data;
        x_err = [drive.bias_x(5) drive.err_x(5)];
        y_err = [drive.bias_y(5) drive.err_y(5)];
        r_err = [drive.bias_r(5) drive.err_r(5)];
    otherwise
        error(strcat('Command ID "', cmd_id,'" not recognized.'))
end

% Break up into the number of segments specified
mov_num = (1:num_segments)';
mov_x = ones(num_segments,1)*x/num_segments;
mov_y = ones(num_segments,1)*y/num_segments;
mov_r = ones(num_segments,1)*r/num_segments;

% Create a dimensionless total movement number for finding errors
mov_d = mov_x + mov_y + mov_r;

% Calculate the errors for each of the segments
movement = [mov_num...
            add_error(mov_d*x_err(1) + mov_d, x_err(2)) - mov_d+mov_x...
            add_error(mov_d*y_err(1) + mov_d, y_err(2)) - mov_d+mov_y...
            add_error(mov_d*r_err(1) + mov_d, r_err(2)) - mov_d+mov_r];

end

