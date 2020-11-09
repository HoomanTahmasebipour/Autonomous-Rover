function plot_clearsense(clearULTRA, clearIR)
%PLOT_CLEARSENSE Summary of this function goes here
%   Detailed explanation goes here
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

global ray_plot
global rayend_plot
global ir_pts
global ir_pts_in
global ir_circle

% Clear Ultrasonic
if clearULTRA
    ray_plot.XData = [];
    ray_plot.YData = [];
    rayend_plot.XData = [];
    rayend_plot.YData = [];
end

% Clear IR
if clearIR
    ir_pts.XData = [];
    ir_pts.YData = [];
    ir_pts_in.XData = [];
    ir_pts_in.YData = [];
    ir_circle.XData = [];
    ir_circle.YData = [];
end

end

