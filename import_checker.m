function [checker_xy] = import_checker(seed, plotmap)
%IMPORT_MAZE This function imports the maze wall locations from a text file
%   Detailed explanation goes here
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

% If no arguments provided, use defaults
if ~exist('seed','var')
    seed = 5489;
end
if ~exist('plotmap','var')
    plotmap = 0;
end

% generate maze checkerboard coordinates
dim1 = 32; dim2 = 16; 
locationindex = reshape(1:dim1*dim2,dim1,dim2)'; 
n = numel(locationindex);
rng(seed,'twister');
checker = reshape(randi([0 1],n,1),dim2,dim1);

% pre-allocate maze variable by defining outer edges, in 3-inch intervals
checker_xy = [NaN, NaN];

% draw boxes representing maze walls
for ct_x = 1:dim1
    for ct_y = 1:dim2
        if checker(ct_y,ct_x) == 0
            coords = [ct_x-1, dim2-ct_y; ...
                      ct_x-1, dim2+1-ct_y; ...
                      ct_x  , dim2+1-ct_y; ...
                      ct_x  , dim2-ct_y; ...
                      ct_x-1, dim2-ct_y; ...
                      NaN   , NaN];
            checker_xy = [checker_xy;coords];
        end
    end
end

% convert the maze coordinates to inches from 3 inch segments
checker_xy = checker_xy*3;

% if plotting output is selected, generate a plot of the maze
if plotmap
    plot(checker_xy(:,1), checker_xy(:,2), 'k')
end

end

