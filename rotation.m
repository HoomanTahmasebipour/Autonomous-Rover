function [vR] = rotation(v, th)
%ROTATION Rotates an input vector about its axis
%   v is the vector, th is the rotation in degrees, vR is
%   the rotated output vector
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

% 2D rotation matrix
R = [cosd(th), sind(th); -sind(th), cosd(th)];

vR = v*R;

end

