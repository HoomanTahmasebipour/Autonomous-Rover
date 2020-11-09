function [cmd_type, cmd_id, cmd_data, id_num] = parse_cmd(cmd,sensor,drive)
%PARSE_CMD determines what type of command has been issued to the robot
%   This function checks the data stored in the sensor and drive structures
%   to try to locate a match to the command issued by the student robot.
%   
%   The output cmd_type is a "0" if the command is not recognized, a "1" if
%   it is a sensor telemetry request, or a "2" if it is a drive command.
%   
%   The output cmd_id is the id of the command
%   
%   ultra - poll ultrasonic sensor
%   ir    - poll downward facing IR sensor
%   comp  - poll compass
%   odom  - poll odometer poll
%   gyro  - poll gyroscope
%   fwd   - move forward
%   back  - move backward
%   left  - move left
%   right - move right
%   rot   - rotate
%   
%   The output id_num is the sensor number to poll (corresponds to the row 
%   number in the 'sensor' or 'drive' variable)
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

if ~exist('cmd','var')
    cmd = 'w1-1';
end

% Strip values from command
cmd_char = cmd(1:2);
cmd_data = str2double(cmd(4:end));
id_num = 0;

% Set cmd_type to default value of 0, in case it isn't found
cmd_type = 0;
cmd_id = 'none';

% Check if this is a sensor request
for ct = 1:size(sensor.char,1)
    if strcmp(cmd_char,sensor.char{ct})
    %if cmd_char == sensor.char{ct}(1)
        %if id_num == str2double(sensor.char{ct}(2))
            cmd_type = 1;
            cmd_id = sensor.id{ct}(1:end-1);
            id_num = ct;
            break
        %end
    end
end

% Check if this is a drive command, but only if a sensor was not recognized
if ~cmd_type
    for ct = 1:size(drive.char,1)
        if strcmp(cmd_char,drive.char{ct})
            cmd_type = 2;
            cmd_id = drive.id{ct};
            id_num = ct;
            break
        end
    end
end

end

