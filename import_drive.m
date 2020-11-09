function [drive] = import_drive(fname)
%IMPORT_DRIVE Imports the drive information from a .csv file
%   Detailed description goes here
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

if ~exist('fname','var')
    fname = 'drive.csv';
end

% Import the sensor data
drive_raw = readtable(strcat('config\',fname), 'Range', 'A2');
drive_mask = drive_raw{:,3};

% Remove the sensors that aren't enabled in the config file
drive_raw(~drive_mask,:) = [];

% Sort the data into a structure
drive.id = drive_raw{:,1};
drive.char = drive_raw{:,2};
drive.err_y = drive_raw{:,4};
drive.err_x = drive_raw{:,5};
drive.err_r = drive_raw{:,6};
drive.bias_y = drive_raw{:,7};
drive.bias_x = drive_raw{:,8};
drive.bias_r = drive_raw{:,9};

end

