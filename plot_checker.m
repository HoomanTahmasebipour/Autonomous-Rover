function [checker_plot] = plot_checker(checker, color, transp)
%PLOT_CHECKER Plots the checkerboard
%   Intakes the NaN-separated checkerboard squares, plots them as a patch
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

if ~exist('checker', 'var')
    load('checker.mat')
end
if ~exist('color', 'var')
    color = 'k';
end
if ~exist('transp', 'var')
    transp = 0.3;
end

% Strip the NaNs from the checker definition array
x = checker(:,1);
y = checker(:,2);
x(isnan(x)) = [];
y(isnan(y)) = [];

% Reshape into columns
x = reshape(x,5,[]);
y = reshape(y,5,[]);

% Plot the checkerboard
checker_plot = patch(x, y, color);
set(checker_plot, 'facealpha', transp)
set(checker_plot, 'LineStyle', 'none')

end

