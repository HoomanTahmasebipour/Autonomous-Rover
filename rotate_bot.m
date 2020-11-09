function [bot_rot, odom, gyro] = rotate_bot(angle, bot_rot, odom_pos, odom, gyro)
%ROTATE_BOT rotates the simulated robot and updates sensor values
%   This function serves to update the rotational value of the robot, and
%   to track the corresponding values of integration-based sensors that are
%   used (odometers and gyroscopes). Though rotation commands may cause
%   movement in x-y axis (errors), all movement errors are pre-calculated
%   in the path_plan function. This function *ONLY* processes rotational
%   movement.
%   
%   Inputs
%   angle - the angle to rotate the robot in degrees
%   bot_rot - the current global orientation of the robot
%   odom_pos - matrix containing the position and orientation of each odometer
%   odom - matrix containing the odometer sensor id number, error term, and current value
%   gyro - same as odom, but for the gyroscope
%   
%   Outputs
%   bot_rot - updated actual orientation angle of the robot
%   gyro - same as above, value updated using this function
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
        % Determine the r vector
        r = [odom_pos(ct,1:2) 0];

        % Determine the unit vector of the direction of the odometer
        th = odom_pos(ct,4);
        v = [cosd(th) sind(th) 0];

        % Calculate the movement of the odometer
        crs = cross(r,v);
        distance = 2*pi*crs(1,3)*angle/360;

        % Add Error to the odometer measurement
        dist_measured = add_error(distance,odom(ct,2));

        % Add to the value in the odometer
        odom(ct,3) = odom(ct,3) + dist_measured;
    end
end

% Update the gyroscope value
if ~isempty(gyro)
    for ct = 1:size(gyro,1)
        angle_measured = add_error(angle, gyro(ct,2));
        gyro(ct,3) = gyro(ct,3) + angle_measured;
    end
end

% Update the robot's rotation
bot_rot = bot_rot + angle;

end

