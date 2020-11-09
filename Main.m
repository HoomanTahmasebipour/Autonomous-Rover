% SimMeR version 0.2
%
% Copyright (c) 2020, Ian G. Bennett
% All rights reserved.
% Development funded by the University of Toronto, Department of Mechanical
% and Industrial Engineering.
% Distributed under GNU AGPLv3 license.

%% Clean up workspace
clear
clear global
close all
clc
disp('Please wait while the simulation loads...')

%% Plotting Variable Initialization
% Global Plotting Variables
global ray_plot
global rayend_plot
global ir_pts
global ir_pts_in
global ir_circle

%% User-editable variables and flags
% Constants
bot_center = [9.5,42];  % Robot starting location
bot_rot = 0;            % Robot starting rotation
block_center = [25,41]; % Block starting location
blocksize = 3;          % Block side length in inches
num_segments = 10;      % Number of movement segments
strength = [0.05, 1];	% How intense the random drive bias is, if enabled
step_time = 0;          % Pause time between the algorithm executing commands

% Control Flags and Setup
randerror = 1;          % Use either a random error generator (1) or consistent error generation (0)
randbias = 1;           % Use a randomized, normally distributed set of drive biases
sim = 1;                % Use the simulator (1) or connect to robot via blueteooth (0)
plot_robot = 1;         % Plot the robot as it works its way through the maze
plot_sense = 1;         % Plot sensor interactions with maze, if relevant

% Bluetooth Serial Connection Constants
comport_num = 6;        % Bluetooth serial comport number to connect to 
comport_baud = 9600;    % Bluetooth serial baudrate

%% Data Import

% Data Import
maze = import_maze;
maze_dim = [min(maze(:,1)), max(maze(:,1)), min(maze(:,2)), max(maze(:,2))];
checker = import_checker;

% Build Block
block = build_block(blocksize, block_center);

% Build Robot
bot_perim = import_bot;
bot_pos = pos_update(bot_center, bot_rot, bot_perim);
bot_front = [0.75*max(bot_perim(:,1)),0];

% Import Sensor Loadout and Positions
sensor = import_sensor;

% Import Drive information
drive = import_drive;

% Initialize integration-based sensor values
gyro_num = [];
odom_num = [];
for ct = 1:size(sensor.id)
    if strcmp('gyro', sensor.id{ct}(1:end-1))
        gyro_num = [gyro_num ct];
    end
    if strcmp('odom', sensor.id{ct}(1:end-1))
        odom_num = [odom_num ct];
    end
end

% Populate the gyroscope and odometer sensor variables
gyro = [gyro_num', sensor.err(gyro_num'), zeros(size(gyro_num))'];
odom = [odom_num', sensor.err(odom_num'), zeros(size(odom_num))'];

%% Act on initialization flags

% Shuffle random number generator seed or set it statically
if randerror
    rng('shuffle') % Use shuffled pseudorandom error generation
else
    rng(0) % Use consistent pseudorandom error generation
end

% Randomize drive biases to verify algorithm robustness
if randbias
    drive = bias_randomize(drive, strength);
end

% Create the plot
if plot_robot
    fig = figure(1);
    axis equal
    hold on
    xlim(maze_dim(1:2))
    ylim(maze_dim(3:4))

    % Maze
    checker_plot = plot_checker(checker);
    maze_plot = plot_checker(maze(7:end,:), 'k', 1);
    plot(maze(:,1),maze(:,2), 'k', 'LineWidth', 2)
    xticks(0:12:96)
    yticks(0:12:48)
    
    % Block
    block_plot = patch(block(:,1),block(:,2), 'y');
    set(block_plot,'facealpha',.5)
    
end

%% Initialize tcp server to read and respond to algorithm commands
clc  % Clear loading message
disp('Simulator initialized... waiting for connection from client')
[s_cmd, s_rply] = tcp_setup('server', 9000, 9001);
fopen(s_cmd);
%fopen(s_rply);

clc
disp('Client connected!')

%% Simulator Main Loop

% Loop Variable Initialization
collision = 0;
bot_trail = [];
firstrun = 1;       % Flag indicating if this is the first time through the loop
firstULTRA = 1;     % Flag indicating if an ultrasonic sensor has been used yet
firstIR = 1;        % Flag indicating if an IR sensor has been used yet

while sim
    
    % Clear the Ultrasonic and IR sensor plots
    if plot_sense
        plot_clearsense(~firstULTRA, ~firstIR)
    end
    
    % Receive data from the algorithm over a TCP socket
    cmd = tcpserver_read(s_cmd);
    
    % Remove the 'newline' character at the end of the character string
    cmd = cmd(1:end-1);
    
    % Parse command
    [cmd_type, cmd_id, cmd_data, id_num] = parse_cmd(cmd, sensor, drive);
    
    % If command is a sensor data request
    if cmd_type == 1
        sensor_pos = [sensor.x(id_num), sensor.y(id_num), sensor.z(id_num), sensor.rot(id_num)];
        pct_error = sensor.err(id_num); % noise value for sensor (from 0 to 1)
        fov = sensor.fov(id_num);
        threshold = sensor.thr(id_num);
        switch cmd_id
            case 'ultra'
                reply = get_ultrasonic(bot_center, bot_rot, sensor_pos, pct_error, fov, maze, block, firstULTRA, plot_sense);
                firstULTRA = 0;
            case 'ir'
                reply = get_ir(bot_center, bot_rot, sensor_pos, pct_error, fov, threshold, checker, firstIR, plot_sense);
                firstIR = 0;
            case 'comp'
                reply = get_compass(bot_center, bot_rot, sensor_pos, pct_error);
            case 'odom'
                reply = get_odometer(odom, id_num);
            case 'gyro'
                reply = get_gyroscope(gyro, id_num);
            otherwise
                error(strcat('Command ID "', cmd_id,'" not recognized.'))
        end
    
    % If command is a movement request
    elseif cmd_type == 2
        
        % Determine the position and rotation of any odometers
        odom_pos = [sensor.x(odom(:,1)), sensor.y(odom(:,1)), sensor.z(odom(:,1)), sensor.rot(odom(:,1))];
        
        % Plan a path with segments for the robot to follow
        movement = path_plan(cmd_id, cmd_data, drive, num_segments);
        
        % Move the robot along the path planned out
        bot_trail = NaN*ones(size(movement,1),2);
        for ct = 1:size(movement,1)
            
            % Set "old position" variables
            odom_old = odom;
            gyro_old = gyro;
            bot_rot_old = bot_rot;
            bot_center_old = bot_center;
            bot_pos_old = bot_pos;
            
            % Rotate the robot
            [bot_rot, odom, gyro] = rotate_bot(movement(ct,4), bot_rot, odom_pos, odom, gyro);
            
            % Move the robot
            [bot_center, odom] = move_bot(movement(ct,2:3), bot_center, bot_rot, odom_pos, odom);
            
            % Update the robot position
            bot_pos = pos_update(bot_center, bot_rot, bot_perim);
            
            % Create a movement path trail for the program to plot
            bot_trail(ct,:) = bot_center;
            
            % Check for any collisions with the maze
            collision = check_collision(bot_pos, maze);
            
            % If there is a collision, reset to old position. Do NOT reset odometers
            if collision
                gyro = gyro_old;
                bot_rot = bot_rot_old;
                bot_center = bot_center_old;
                bot_pos = bot_pos_old;
                bot_trail(ct,:) = [NaN NaN];
            end
            
        end
        
        % Use Inf to stand in for "movement complete"
        reply = Inf;
        
    else
        disp(strcat('Unrecognized command issued or sensor "',cmd(1:2),'" not found.'));
        % Use NaN to stand in for "Unrecognized command"
        reply = NaN;
    end
    
    % After executing the requested command, plot the result
    if (plot_robot || collision)
        
        % Robot
        if firstrun
            robot = patch(bot_pos(:,1),bot_pos(:,2), 'g');
            set(robot,'facealpha',.5)
        else
            tmpx = bot_pos(:,1);
            tmpy = bot_pos(:,2);
            robot.XData = tmpx;
            robot.YData = tmpy;
        end
        
        % Robot "nose"
        nose = pos_update(bot_center, bot_rot, bot_front);
        if firstrun
            robot_fwd = plot(nose(1), nose(2), 'k*');
            robot_fwd.XDataSource = 'nose(1)';
            robot_fwd.YDataSource = 'nose(2)';
        end
        
        % Robot Movement
        if ~isempty(bot_trail)
            if ~exist('robot_trail', 'var')
                robot_trail = plot(bot_trail(:,1), bot_trail(:,2), 'r*-');
                robot_trail.XDataSource = 'bot_trail(:,1)';
                robot_trail.YDataSource = 'bot_trail(:,2)';
            end
        end
        refreshdata
    end
    
    % Wait before allowing the algorithm to continue
    pause(step_time)
    
    % Return the reply variable to the user algorithm
    %fwrite(s_rply, reply, 'single')
    fwrite(s_cmd, reply, 'single')
    
    % If not the first run of the loop, set flag to 0
    firstrun = 0;
    
end

%% Bluetooth Main Loop
if ~sim

% Initialize the bluetooth serial port
b_com = serialport(comport_num, comport_baud);

while ~sim
    
    
    
end
end
