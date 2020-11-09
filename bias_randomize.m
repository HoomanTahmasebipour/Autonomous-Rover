function [drive] = bias_randomize(drive, strength)
%BIAS_RANDOMIZE generates randomized drive biases
%   Detailed description goes here
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

% Set variables if not set by function call
if ~exist('drive','var')
    load('drive.mat');
end
if ~exist('strength','var')
    strength = [0.05, 0.05];
end

% Movement biases
for ct = 1:(length(drive.bias_x)-1)
    drive.bias_x(ct,1) = randn * strength(1);
    drive.bias_y(ct,1) = randn * strength(1);
    drive.bias_r(ct,1) = randn * strength(1) * 10;
end

% Rotation bias
ct = length(drive.bias_x);
drive.bias_x(ct,1) = randn * strength(2) / 360;
drive.bias_y(ct,1) = randn * strength(2) / 360;
drive.bias_r(ct,1) = randn * strength(2) / 360;

end

