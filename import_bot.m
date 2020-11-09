function [bot_xy] = import_bot(fname, plotmap)
%IMPORT_BOT imports and defines robot perimeter coordinates
%   Detailed explanation goes here
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

if ~exist('fname','var')
    fname = 'robot.csv';
end
if ~exist('plotmap','var')
    plotmap = 0;
end

% import robot size definition
bot_params = csvread(strcat('config\',fname),0,1);

% Verify that the robot is smaller than 1 foot by 1 foot
if (bot_params(2) > 12 || bot_params(3) > 12)
    error(strcat('Imported robot is too large, it must be less than 12" x 12". Check',' config\',fname,' to verify dimensions.'));
end
    

if ~bot_params(1)
    d = bot_params(2);
    th = fliplr(0:pi/20:2*pi);
    x = d/2 * cos(th);
    y = d/2 * sin(th);
    bot_xy = [x',y'];
else
    x = bot_params(2);
    y = bot_params(3);
    bot_xy = [-x/2, -y/2; ...
              -x/2,  y/2; ...
               x/2,  y/2; ...
               x/2, -y/2; ...
              -x/2, -y/2];
end

if plotmap
    plot(bot_xy(:,1), bot_xy(:,2))
end

end