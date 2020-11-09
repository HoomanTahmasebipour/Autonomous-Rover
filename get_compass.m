function [direction] = get_compass(bot_center, bot_rot, sensor_pos, pct_error)
%GET_COMPASS Returns the output of a simulated compass
%   Detailed explanation goes here
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

% Constants

% Determine sensor absolute position
origin = pos_update(bot_center, bot_rot, sensor_pos(1:2));

% Determine sensor absolute orientation
direction_pure = sensor_pos(4) + bot_rot;

% Calculate the error and add it to the measurement
direction = add_error(direction_pure, pct_error);

end

