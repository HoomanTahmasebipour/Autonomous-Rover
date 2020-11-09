function [pos] = pos_update(origin, rot, pos_relative)
%POS_UPDATE converts a relative bot/sensor position to an absolute one
%   This function converts a relative position vector to an absolute one
%   based on an origin position and rotation value.
%   
%   The origin position (origin) should be a two element vector [x, y]
%   containing the absolue positionof the origin.
%   
%   The rotation vector (rot) should be a scalar indicating the rotation
%   angle in degrees.
%   
%   The relative position (pos_relative) should be a matrix with each row
%   containing a set of coordinates [x, y].
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

% Rotate vector
pos = rotation(pos_relative, rot);
pos = repmat(origin, size(pos_relative,1), 1) + pos;

end