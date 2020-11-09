function [bot_center, odom] = move_bot(pos, bot_center, bot_rot, odom_pos, odom)
%MOVE_BOT moves the robot center position in the x-y axis
%   This function serves to update the rotational value of the robot, and
%   to track the corresponding values of integration-based sensors that are
%   used (odometers and gyroscopes). Though rotation commands may cause
%   movement in x-y axis (errors), all movement errors are pre-calculated
%   in the path_plan function. This function *ONLY* processes rotational
%   movement.
%   
%   Inputs
%   pos - the x-y movement of the robot in inches
%   bot_center - the current global position of the robot
%   bot_rot - the current global orientation of the robot
%   odom_pos - matrix containing the position and orientation of each odometer
%   odom - matrix containing the odometer sensor id number, error term, and current value
%   
%   Outputs
%   bot_center - updated global position of the robot
%   odom - same as above, value updated using this function
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.


% Update the odometer value (working in the local coordinate system)
if ~isempty(odom)
    for ct = 1:size(odom_pos,1)
        % Determine the unit vector of the direction the odometer points
        th = odom_pos(ct,4);
        v = [cosd(th) sind(th) 0];

        % Calculate the movement of the odometer
        distance = dot(pos,v(1:2));

        % Add Error to the odometer measurement
        dist_measured = add_error(distance,odom(ct,2));

        % Add to the value in the odometer
        odom(ct,3) = odom(ct,3) + dist_measured;
    end
end


% Update the robot position after transforming to a local coordinate system
bot_center = pos_update(bot_center, bot_rot, pos);

end

