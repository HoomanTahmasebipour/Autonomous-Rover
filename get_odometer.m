function [value] = get_odometer(odom, id_num)
%GET_ODOMETER reads the selected stored odometer value
%   Reads the stored odometer value and formats it in a way that can be
%   interpreted by the control algorithm
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

value = odom(odom(:,1)==id_num,3);
 
end

