function [sensor] = import_sensor(bot_perim, fname, plotmap)
%IMPORT_SENSOR Imports the sensor loadout
%   This function reads the sensor csv file and loads the provided
%   information into the program. Data is stored in a 1x1 structure
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

if ~exist('bot_perim','var')
    bot_perim = [-6, -6; ...
                 -6,  6; ...
                  6,  6; ...
                  6, -6; ...
                 -6, -6];
end
if ~exist('fname','var')
    fname = 'sensors.csv';
end
if ~exist('plotmap','var')
    plotmap = 0;
end

% Import the sensor data
sensor_raw = readtable(strcat('config\',fname), 'Range', 'A2');
sensor_mask = sensor_raw{:,3};

% Remove the sensors that aren't enabled in the config file
sensor_raw(~sensor_mask,:) = [];

% Sort the data into a structure
sensor.id = sensor_raw{:,1};
sensor.char = sensor_raw{:,2};
sensor.x = sensor_raw{:,4};
sensor.y = sensor_raw{:,5};
sensor.z = sensor_raw{:,6};
sensor.rot = sensor_raw{:,7};
sensor.err = sensor_raw{:,8};
sensor.fov = sensor_raw{:,9};
sensor.thr = sensor_raw{:,10};

% Plot the positions of the sensors on the robot if needed
if plotmap == 1
    figure(1)
    hold on
    plot(bot_perim(:,1), bot_perim(:,2))
    for ct = 1:size(sensor.x,1)
        plot(sensor.x(ct), sensor.y(ct), 'o')
    end
    legend(['Robot outline'; sensor.id])
end

end

