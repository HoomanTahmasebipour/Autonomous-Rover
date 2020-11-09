function [maze_xy] = import_maze(fname, plotmap)
%IMPORT_MAZE This function imports the maze wall locations from a text file
%   Detailed explanation goes here
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

% If no arguments provided, use defaults
if ~exist('fname','var')
    fname = 'maze.csv';
end
if ~exist('plotmap','var')
    plotmap = 0;
end

% import maze wall coordinates
maze = csvread(strcat('config\',fname));

% pre-allocate maze variable by defining outer edges, in feet
dim1 = 8; dim2 = 4;
maze_xy = [NaN, NaN;...
           0  , 0; ...
           dim1  , 0; ...
           dim1  , dim2; ...
           0  , dim2; ...
           0  , 0; ...
           NaN, NaN];

% draw boxes representing maze walls
for ct_x = 1:dim1
    for ct_y = 1:dim2
        if maze(ct_y,ct_x) == 0
            coords = [ct_x-1, dim2-ct_y; ...
                      ct_x-1, dim2+1-ct_y; ...
                      ct_x  , dim2+1-ct_y; ...
                      ct_x  , dim2-ct_y; ...
                      ct_x-1, dim2-ct_y; ...
                      NaN   , NaN];
            maze_xy = [maze_xy;coords];
        end
    end
end

% convert the maze coordinates to inches from feet
maze_xy = maze_xy*12;

% if plotting output is selected, generate a plot of the maze
if plotmap
    plot(maze_xy(:,1), maze_xy(:,2), 'k')
end

end

