clear
close all
clc

%% Request User Decision on which Dropoff location to go to
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Requesting Input By Default %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp("Drop off location '1': (3, 3)")
disp("Drop off location '2': (1, 6)")
disp("Drop off location '3': (1, 8)")
disp("Drop off location '4': (4, 8)")

drop_off_id = input("From the above options, which drop off location would you like to go to? (Only 1,2,3,4 accepted as inputs)\n");
if (drop_off_id == 1)
    disp("Going to drop off location 1. Rover starting.....")
elseif (drop_off_id == 2)
    disp("Going to drop off location 2. Rover starting.....")
elseif (drop_off_id == 3)
    disp("Going to drop off location 3. Rover starting.....")
elseif (drop_off_id == 4)
    disp("Going to drop off location 4. Rover starting.....")
else
    disp("Invalid option selected. Going to default drop off location 1")
    drop_off_id = 1;
end

%% Rover/Simulation Initialization and Setup
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Set sim variable to 0 for testing with Rover %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sim = 1;

if sim
% Initialize tcp server to read and respond to algorithm commands
[s_cmd, s_rply] = tcp_setup();
fopen(s_cmd);
%fopen(s_rply);
else
    %connect to rover Bluetooth
end

% Global Variables
u = [0,0,0,0,0,0];
u_loc = [3.09,-1.56 ; 1.13,3.52 ; -3.49,1.04 ; -2.34375,-3.43 ; 2.34375,-3.43 ; 2.68,3.52];
pos = [0,0,0];  % Position (x,y,rotation)
rot_stuck = 90;
stepcount = 0;

rover_radius = 4;
rover_dist_thresh = 2;
ultrasonic_margin = 0.3;

%% Localization - create world and initialize probability
%provide measurements and movements
k = 0;

%initalization of the world
dim1 = 32; dim2 = 16; 
locationindex = reshape(1:dim1*dim2,dim1,dim2)'; 
n = numel(locationindex);
rand('twister',5489);
bw = reshape(randi([0 1],n,1),dim2,dim1); %0 = black, 1 = white

%make blocks
M = zeros(size(bw));
Blocks = [2, 3; 3, 2; 4, 3; 5, 1; 5, 3; 7, 1; 7, 3; 7, 4;];
for xx = 1:size(Blocks,1),
	x = Blocks(xx,1); y = Blocks(xx,2);
	M(1+(y-1)*4:(y-1)*4+4, 1+(x-1)*4:(x-1)*4+4) = 1;
end
M = [ones(dim2,1) M ones(dim2,1)];
M = [ones(1, dim1+2); M; ones(1, dim1+2)];

%generate ultrasonic world
ultra = zeros(size(bw));
for sec_row = 1:4:dim2,
    for sec_col = 1:4:dim1,
        segRow = M(sec_row+2, sec_col:sec_col+5);
        segCol = M(sec_row:sec_row+5, sec_col+2);
        val = sum(segRow)+sum(segCol);
        if val == 2 && sum(segRow)~=1,
            val = 5;
        end
        ultra(sec_row:sec_row+3, sec_col:sec_col+3) = val;
    end
end

%create mask for blocks
M = abs(M-1);
M = M(2:end-1, 2:end-1);
figure; imagesc((bw+1).*M); colormap(gray);

%initialize probability
p = ones(dim2,dim1)*(1/n); 

figure;

%% Build Navigation Matrices
loading_zone_nav = [0  0   1  2  inf  6 inf 8; 
                    0  0  inf 3   4   5  6  7; 
                    1 inf  5 inf inf  6 inf 8; 
                    2  3   4  5   6   7 inf 9];

drop_off_1_nav = [6  7   8  9  inf  7 inf 9; 
                  5 inf inf 8   7   6  7  8; 
                  4 inf  0 inf inf  5 inf 9; 
                  3  2   1  2   3   4 inf 10];
                
drop_off_2_nav = [7  6   5  4  inf  0 inf 4; 
                  8 inf inf 3   2   1  2  3; 
                  9 inf  7 inf inf  2 inf 4; 
                  8  7   6  5   4   3 inf 5];
                
drop_off_3_nav = [9   8   7   6  inf  4 inf 0; 
                  10 inf  inf  5   4   3  2  1; 
                  11 inf  9  inf inf  4 inf 2; 
                  10  9   8   7   6   5 inf 3];
                
drop_off_4_nav = [10   9   8   7  inf  5 inf 3; 
                  11 inf  inf  6   5   4  3  2; 
                  12 inf  10  inf inf  5 inf 1; 
                  11  10   9   8   7   6 inf 0];

%% Main Rover Control Code: Localize, navigate to loading zone, navigate to drop off zone
heading = straighten_and_find_heading(u, sim, s_cmd, s_rply, rover_radius, u_loc);
[heading, p, k, M, ultra] = ultrasonic_localization(u_loc, s_cmd, s_rply, heading, u, p, k, M, ultra, rover_radius, rover_dist_thresh, ultrasonic_margin);
[u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
[p,k,loc_y,loc_x, localized, max_prob] = update_rover_location(p, M, heading, k);
p = update_localization_map(u, M, p, ultra, k);
[tile_row, tile_col] = determine_rover_tile_loc(loc_x, loc_y);
disp(['Rover has determined it is in tile: (' num2str(tile_row) ', ' num2str(tile_col) ')'])
disp("------------------------- -----------------------------------------------")

% Drive the Rover to the Loading Zone
disp("Rover is now driving to the loading zone")
lz = 1;
[heading, p, k, M, ultra] = drive_to_destination(s_cmd, s_rply, heading, loading_zone_nav, loc_x, loc_y, u_loc, rover_radius, M, p, ultra, k, ultrasonic_margin, drop_off_id, lz);
[u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
[p,k,loc_y,loc_x, localized, max_prob] = update_rover_location(p, M, heading, k);
p = update_localization_map(u, M, p, ultra, k);
[tile_row, tile_col] = determine_rover_tile_loc(loc_x, loc_y);
disp(['Rover has arrived at the loading zone, in tile: (' num2str(tile_row) ', ' num2str(tile_col) ')'])

% Find and load the block
disp('Rover is now searching for the block')
[heading, p, k, M, ultra, loc_x, loc_y] = find_and_load_block(s_cmd, s_rply, rover_radius, u_loc, rover_dist_thresh, p, k, M, ultra, heading, ultrasonic_margin);
[u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
[p,k,loc_y,loc_x, localized, max_prob] = update_rover_location(p, M, heading, k);
p = update_localization_map(u, M, p, ultra, k);
[tile_row, tile_col] = determine_rover_tile_loc(loc_x, loc_y);
disp('Block found and loaded!')
disp(['Rover is currently in tile: (' num2str(tile_row) ', ' num2str(tile_col) ')'])

% Drive the Rover to Delivery point
lz = 0;
disp(['Rover is now driving to drop off zone with ID: ' num2str(drop_off_id)])
if (drop_off_id == 1)
    [heading, p, k, M, ultra] = drive_to_destination(s_cmd, s_rply, heading, drop_off_1_nav, loc_x, loc_y, u_loc, rover_radius, M, p, ultra, k, ultrasonic_margin, drop_off_id, lz);
elseif (drop_off_id == 2)
    [heading, p, k, M, ultra] = drive_to_destination(s_cmd, s_rply, heading, drop_off_2_nav, loc_x, loc_y, u_loc, rover_radius, M, p, ultra, k, ultrasonic_margin, drop_off_id, lz);
elseif (drop_off_id == 3)
    [heading, p, k, M, ultra] = drive_to_destination(s_cmd, s_rply, heading, drop_off_3_nav, loc_x, loc_y, u_loc, rover_radius, M, p, ultra, k, ultrasonic_margin, drop_off_id, lz);
elseif (drop_off_id == 4)
    [heading, p, k, M, ultra] = drive_to_destination(s_cmd, s_rply, heading, drop_off_4_nav, loc_x, loc_y, u_loc, rover_radius, M, p, ultra, k, ultrasonic_margin, drop_off_id, lz);
end

%% Exit Sequence
[u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
[p,k,loc_y,loc_x, localized, max_prob] = update_rover_location(p, M, heading, k);
p = update_localization_map(u, M, p, ultra, k);
[tile_row, tile_col] = determine_rover_tile_loc(loc_x, loc_y);
disp(['Rover has arrived at the drop off zone with ID: ' num2str(drop_off_id) ', in tile: (' num2str(tile_row) ', ' num2str(tile_col) ')'])

% Unload the block
deliver_block_and_close_gate(s_cmd, s_rply, rover_radius, u_loc, ultrasonic_margin, heading)
disp('%%%%%%%%%%%%%%% Block has been delivered! %%%%%%%%%%%%%%%%%%')
%% Helper functions
function heading = straighten_and_find_heading(u, sim, s_cmd, s_rply, rover_radius, u_loc)
    straighten_attempts = 0;
    unique_loc = 0;
    rover_straight = 0;

    % straighten rover and take compass readings to determine heading
    while rover_straight == 0
        % Rotate rover until sensor measurements from u4/u5 are
        % within an acceptable range
        [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);

        if (straighten_attempts < 5)
            rover_straight = straighten_rover(u, s_cmd, s_rply, unique_loc);
            straighten_attempts = straighten_attempts + 1;
        else
            straighten_attempts = 0;
            rover_straight = 1;
        end
    end

    comp = tcpclient_write(['c1' newline], s_cmd, s_rply);
    disp(comp)
    if (sim == 1)
        if comp < 45 && comp > -45
            heading = 0;
        elseif comp < 135 && comp > 45
            heading = 90;
        elseif comp < -135 || comp > 135
            heading = 180;
        elseif comp > -135 && comp < -45
            heading = 270;
        end
    else
        if comp < 45 || comp > 315
            heading = 0;
        elseif comp < 135 && comp > 45
            heading = 90;
        elseif comp < 225 && comp > 135
            heading = 180;
        elseif comp < 315 && comp > 225
            heading = 270;
        end 
    end
    disp(['Heading: ' num2str(heading)])
end

function [heading, p, k, M, ultra] = ultrasonic_localization(u_loc, s_cmd, s_rply, heading, u, p, k, M, ultra, rover_radius, rover_dist_thresh, ultrasonic_margin)
    rover_centered = 0;
    straighten_attempts = 0;
    center_attempts = 0;
    unique_loc = 0;
    first_unique_loc = 0;
    localized = 0;
    old_heading = heading;
    [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
    disp('Ultrasonic Measurements')
    disp(u)
        
    while (localized == 0)    
        % Take Measurements (read from simulator or rover)
        disp(['------------------------- Step Count: ' num2str(k) ' -------------------------'])
        %[u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
        %disp('Ultrasonic Measurements')
        %disp(u)

        if (straighten_attempts < 5 && old_heading ~= heading)
            rover_straight = straighten_rover(u, s_cmd, s_rply, unique_loc);
            if (rover_straight == 0)
                straighten_attempts = straighten_attempts + 1;
            else
                straighten_attempts = 0;
            end
        else
            straighten_attempts = 0;
            rover_straight = 1;
        end

        unique_loc = 0;
        if (u_real(1) > 10 && u_real(2) > 10 && u_real(3) > 10 && (u_real(4) > 10 || u_real(5) > 10))
            unique_loc = 1;
            if (first_unique_loc == 0)
                first_unique_loc = 1;
            else
                first_unique_loc = 0;
            end
            [p,k,loc_y,loc_x, localized, max_prob] = update_rover_location(p, M, heading, k);
            p = update_localization_map(u, M, p, ultra, k);
            disp('unique_loc')
        end

        if (rover_straight == 1 && center_attempts < 2)
            rover_centered = center_rover(u, s_cmd, s_rply, ultrasonic_margin, rover_radius, unique_loc, first_unique_loc, heading);
            if (first_unique_loc == 1)
                rover_centered = 0;
                unique_loc = 0;
            end
            if (rover_centered == 0)
                center_attempts = center_attempts + 1;
            else
                center_attempts = 0;
            end
        else
            rover_centered = 1;
            center_attempts = 0;
        end
        if ((rover_centered == 1 && rover_straight == 1) || unique_loc == 1)
            old_heading = heading;
            heading = move_rover(u, s_cmd, s_rply, rover_dist_thresh, heading);
        end
        
        [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
        disp('Ultrasonic Measurements')
        disp(u)
        if (rover_straight == 1)
            [p,k,loc_y,loc_x, localized, max_prob] = update_rover_location(p, M, heading, k);
            p = update_localization_map(u, M, p, ultra, k);
        end
    end
    heading = heading;
end

function [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc)
    % This function takes ultrasonic readings as many as num_measurements,
    % averages them out, and returns them in a 1x6 vector.
    u = [0,0,0,0,0,0];
    u_real = [0,0,0,0,0,0];
    cmdstring = ['ua-5' newline];
    u_real = tcpclient_write(cmdstring, s_cmd, s_rply);
    u1 = u_real(1) - (rover_radius - abs(u_loc(1,1)));
    u2 = u_real(2) - (rover_radius - abs(u_loc(2,2)));
    u3 = u_real(3) - (rover_radius - abs(u_loc(3,1)));
    u4 = u_real(4) - (rover_radius - abs(u_loc(4,2)));
    u5 = u_real(5) - (rover_radius - abs(u_loc(5,2)));
    u6 = u_real(6) - (rover_radius - abs(u_loc(6,1)));
    u = [u1 u2 u3 u4 u5 u6];
end

function rover_straight = straighten_rover(u, s_cmd, s_rply, unique_loc)
    % This function ensures that after each step of the rover, it is
    % continuing to move straight
    lower_thresh_straight = 0.2;
    upper_thresh_straight = 5;
    u4_u5_sensor_displacement = 4.6875;
    u4_u5_diff = abs(u(4) - u(5));
        
    if (unique_loc == 0 && (u4_u5_diff < lower_thresh_straight || u4_u5_diff > upper_thresh_straight))
        rover_straight = 1;
        disp('Rover is straight')
    elseif (unique_loc == 1)
        rover_straight = 1;
        disp('Unique location, assuming rover is straight')
    else
        % Rover is not straight
        disp('Rover is not straight')
        rover_straight = 0;
        % Determine number of degrees to turn rover
        rot = atand(u4_u5_diff / u4_u5_sensor_displacement) / 2;
                
        % Determine if to turn rover CW or CCW
        if (u(4) > u(5))
            disp(['Straighten rover: turn in postive direction by: ' num2str(rot)])
            cmd = strcat('r1-',num2str(rot)); 
            cmdstring = [cmd newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (u(4) <= u(5))
            disp(['Straighten rover: turn in negative direction by: ' num2str(rot)])
            cmd = strcat('r1--',num2str(rot));
            cmdstring = [cmd newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    end 
end

function rover_centered = center_rover(u, s_cmd, s_rply, ultrasonic_margin, rover_radius, unique_loc, first_unique_loc, heading)
    % This function centers the rover as much as possible to ensure
    % straight motion
    u_left = u(2);
    if (u(4) < u(5))
        u_right = u(4);
    else
        u_right = u(5);
    end
    centered_dist_from_wall = 6 - rover_radius;
    centered_dist_from_wall_min = centered_dist_from_wall - centered_dist_from_wall * ultrasonic_margin;
    centered_dist_from_wall_max = centered_dist_from_wall + centered_dist_from_wall * ultrasonic_margin;
        
    if (u(1) < centered_dist_from_wall_max || (unique_loc == 0 && ((u_left > centered_dist_from_wall_min && u_left < centered_dist_from_wall_max) || (u_right > centered_dist_from_wall_min && u_right < centered_dist_from_wall_max))))
        rover_centered = 1;
        disp('rover_centered')
    elseif (unique_loc == 0 && (u_left < centered_dist_from_wall_min || (u_right < u_left && u_right > centered_dist_from_wall_max)))
        % Either left side is too close or right side is too far, move right
        angle = 22;
        adjustment = u_right / 2;
        req_dist = adjustment / tand(angle);
        speed = adjustment / sind(angle);
        if (speed > 3)
            speed = 3;
        end
        if (u(1) > req_dist)
            cmdstring = [strcat('r1-',num2str(-angle)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('r1-',num2str(angle)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            disp(['Left/Right side too close/far, move right. r1-22, d1-' num2str(speed) ', r1--22'])
            rover_centered = 0;
        else
            rover_centered = 1;
            disp('Left/Right side too close/far but cant move to fix. Moving back half an inch and assuming centered')
            cmdstring = [strcat('a1-',num2str(0.5)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (unique_loc == 0 && (u_right < centered_dist_from_wall_min || (u_left < u_right && u_left > centered_dist_from_wall_max)))
        % Eithert right side is too close or left side is too far, move left
        angle = 22;
        adjustment = u_left / 2;
        req_dist = adjustment / tand(angle);
        speed = adjustment / sind(angle);
        if (speed > 3)
            speed = 3;
        end
        if (u(1) > req_dist) 
            cmdstring = [strcat('r1-',num2str(angle)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('r1-',num2str(-angle)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            disp(['Right/Left side too close/far, move left. r1--22, d1-' num2str(speed) ', r1-22'])
            rover_centered = 0;
        else
            rover_centered = 1;
            disp('Right/Left side too close/far but cant move to fix. Moving back half an inch and assuming centered')
            cmdstring = [strcat('a1-',num2str(0.5)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (unique_loc == 1)
        rover_centered = 1;
        if (first_unique_loc == 1)
            disp('Rover in unique location, centering...')
            center_unique_loc(u, s_cmd, s_rply, heading);
        else
            disp('Exiting unique location')
        end
    else
        rover_centered = 1;
    end
end

function center_unique_loc(u, s_cmd, s_rply, heading)
    if (heading == 0 || heading == 180)
        % u1 ~= u3 ~= 26
        if (u(3) < 25.5 && u(1) > 28)
            % u(3) too close and u(1) too far, move in u(1) direction
            % determine speed by finding out how far off you are, and
            % add a 0.5 margin for safety
            speed = 25.5 - u(3) + 0.5;
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (u(1) < 26 && u(3) > 27.5)
            % u(3) too far and u(1) too close, move in u(3) direction
            % (reverse), determine speed by finding out how far off you are, and
            % add a 0.5 margin for safety
            speed = 26 - u(1) + 0.5;
            cmdstring = [strcat('a1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (heading == 90)
        % u(3) ~= 26 & u(1) ~ 14
        if (u(3) < 25.5 && u(1) > 16)
            % u(3) too close and u(1) too far, move in u(1) direction
            % determine speed by finding out how far off you are, and
            % add a 0.5 margin for safety
            speed = 25.5 - u(3) + 0.5;
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (u(1) < 14 && u(3) > 27.5)
            % u(1) too close and u(3) too far, move in u(3) direction
            % determine speed by finding out how far off you are, and
            % add a 0.5 margin for safety
            speed = 14 - u(1) + 0.5;
            cmdstring = [strcat('a1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (heading == 270)
        % u(1) ~= 14 & u(3) ~= 26
        if (u(3) < 13.5 && u(1) > 28)
            % u(3) too close and u(1) too far, move in u(1) direction
            % determine speed by finding out how far off you are, and
            % add a 0.5 margin for safety
            speed = 13.5 - u(3) + 0.5;
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (u(1) < 26 && u(3) > 15.5)
            % u(1) too close and u(3) too far, move in u(3) direction
            % determine speed by finding out how far off you are, and
            % add a 0.5 margin for safety
            speed = 26 - u(1) + 0.5;
            cmdstring = [strcat('a1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    end
end

function heading = move_rover(u, s_cmd, s_rply, rover_dist_thresh, heading)
    % This function's purpose is to move the rover one step in the optimal
    % direction
    u45 = abs(u(4) + u(5))/2;
        
    if (u(2) > u45 && u(2) > rover_dist_thresh*1.5 && u(1) <= rover_dist_thresh*1.25)
        % Move left, first determine rover speed
        speed = u(2) / 2;
        if (speed > 6)
            speed = 6;
        end
        
        % Rotate rover 90 CW and move forward one step
        cmdstring = [strcat('r1-',num2str(90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp(['Move left ' num2str(speed) ' inches'])
        heading = mod(heading + 90,360);
    elseif (u45 >= u(2) && u45 >= rover_dist_thresh*1.5 && u(1) <= rover_dist_thresh*1.25)
        % Move right, first determine rover speed
        if (u(4) <= u(5))
            speed = u(4) / 2;
        elseif (u(5) < u(4))
            speed = u(5) / 2;
        end
        if (speed > 6)
            speed = 6;
        end
        
        % Rotate rover 90 CCW and move forward one step
        cmdstring = [strcat('r1-',num2str(-90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp(['Move right ' num2str(speed) ' inches'])
        heading = mod(heading - 90,360);
    elseif (u(1) > rover_dist_thresh*1.25)
        % Move forward, first determine rover speed
        speed = u(1) / 2;
        if (speed > 6)
            speed = 6;
        end
        
        % Move rover forward one step
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp(['Move forward ' num2str(speed) ' inches'])
        heading = heading;
    elseif (u(3) > rover_dist_thresh)
        % Move backwards, first determine rover speed
        speed = u(3) /2;
        if (speed > 6)
            speed = 6;
        end
        
        % Rotate rover 180 CW and move forward one step
        cmdstring = [strcat('r1-',num2str(180)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp(['Reverse ' num2str(speed) ' inches'])
        heading = mod(heading + 180,360);
    end
end

function p = update_localization_map(u, M, p, ultra, k)
    % Determine rover location
    if (u(4) <= u(5))
        u_right = u(4);
    else
        u_right = u(5);
    end
    u_temp = [u(1) u(2) u(3) u_right];
    u_dig = u_temp < 12;
    disp('u_dig')
    disp(u_dig)
    m_u = sum(u_dig);
    if m_u == 2 && (u_dig(1) == u_dig(3) || u_dig(2) == u_dig(4))
        m_u = 5;
    end
    
    %sensor update
    p = sense_u(ultra, M, p, m_u);
    
    imagesc(p);
    colorbar
    title(['step: ' num2str(k)]);
    pause(0.5);
end

function [p,k,loc_y,loc_x, localized, max_prob] = update_rover_location(p, M, heading, k)
    % movement update
    p = move(p, M, heading);
    
    disp(['heading: ' num2str(heading)])
    k = k + 1;

    % determine index location
    %[loc_y, loc_x] = find(p == max(p(:)));
    [max_prob, index] = max(p(:));
    [loc_y,loc_x] = ind2sub(size(p),index);
    disp('Possible location of rover (x, y):')
    disp([loc_x loc_y max_prob])
    if (max_prob >= 0.085)
        localized = 1;
    else
        localized = 0;
    end
end

function pnew = move(p, mask, heading)
   %models movement error
   K = [0.1 0.1 0.1; 0.1 0.8 0.1; 0.1 0.1 0.1];
   
   %2D convolution
   pnew = conv2(p, K,'same');
   
   col_move = cosd(heading);
   row_move = sind(heading);
   
   if col_move > 0
	   pnew = [zeros(size(pnew,1),1) pnew(:,1:end-1)];
   elseif col_move < 0
	   pnew = [pnew(:,2:end) zeros(size(pnew,1),1)];
   end
   
   if row_move < 0
	   pnew = [zeros(1,size(pnew,2)); pnew(1:end-1,:)];
   elseif row_move > 0
	   pnew = [pnew(2:end,:); zeros(1,size(pnew,2))];
   end

   %masking matrix
   pnew = pnew.*mask;

   %normalization
   pnew = pnew/sum(sum(pnew));
  
end

function pnew = sense_u(world, mask, p, SenVal)
  %models sensor reading
  pHit = 0.6; %default = 0.9
  pMiss = 0.2; %default = 0.1
  
  mult = pMiss*ones(size(world));
  mult(world == SenVal) = pHit;
  
  %2D multiplication step
  pnew = p.*mult;
  
  %masking matrix
  pnew = pnew.*mask;
   
  %normalization
  pnew = pnew./sum(sum(pnew));

end

function [tile_row, tile_col] = determine_rover_tile_loc(loc_x, loc_y)
    % loc_x is the coloumn of the matrix, loc_y is the row of the matrix
    if (loc_x <= 4 && loc_x >= 1)
        tile_col = 1;
    elseif (loc_x <= 8 && loc_x > 4)
        tile_col = 2;
    elseif (loc_x <= 12 && loc_x > 8)
        tile_col = 3;
    elseif (loc_x <= 16 && loc_x > 12)
        tile_col = 4;
    elseif (loc_x <= 20 && loc_x > 16)
        tile_col = 5;
    elseif (loc_x <= 24 && loc_x > 20)
        tile_col = 6;
    elseif (loc_x <= 28 && loc_x > 24)
        tile_col = 7;
    elseif (loc_x <= 32 && loc_x > 28)
        tile_col = 8;
    else
        tile_col = 0;
    end
    
    if (loc_y <= 4 && loc_y >= 1)
        tile_row = 1;
    elseif (loc_y <= 8 && loc_y > 4)
        tile_row = 2;
    elseif (loc_y <= 12 && loc_y > 8)
        tile_row = 3;
    elseif (loc_y <= 16 && loc_y > 12)
        tile_row = 4;
    else
        tile_row = 0;
    end
end

function [new_heading, new_tile_row, new_tile_col] = determine_new_heading(nav_grid, tile_row, tile_col)
    % This function determines the direction the rover should try to move
    % in. 1 means north, 2 means east, 3 means south, 4 means west
    if (tile_row < 4 && tile_row > 1)
        north_tile = tile_row - 1;
        south_tile = tile_row + 1;
    elseif (tile_row == 4)
        north_tile = tile_row - 1;
        south_tile = tile_row;
    elseif (tile_row == 1)
        north_tile = tile_row;
        south_tile = tile_row + 1;
    else
        north_tile = tile_row;
        south_tile = tile_row;
    end

    if (tile_col < 8 && tile_col > 1)
        east_tile = tile_col + 1;
        west_tile = tile_col - 1;
    elseif (tile_col == 8)
        east_tile = tile_col;
        west_tile = tile_col - 1;
    elseif (tile_col == 1)
        east_tile = tile_col + 1;
        west_tile = tile_col;
    else
        east_tile = tile_col;
        west_tile = tile_col;
    end
    
    north_dir_val = nav_grid(north_tile, tile_col);
    south_dir_val = nav_grid(south_tile, tile_col);
    east_dir_val = nav_grid(tile_row, east_tile);
    west_dir_val = nav_grid(tile_row, west_tile);
    
    if (north_dir_val <= south_dir_val && north_dir_val <= east_dir_val && north_dir_val <= west_dir_val)
        new_heading = 90;
        new_tile_row = north_tile;
        new_tile_col = tile_col;
    elseif (south_dir_val < north_dir_val && south_dir_val < east_dir_val && south_dir_val < west_dir_val)
        new_heading = 270;
        new_tile_row = south_tile;
        new_tile_col = tile_col;
    elseif (east_dir_val <= south_dir_val && east_dir_val < north_dir_val && east_dir_val < west_dir_val)
        new_heading = 0;
        new_tile_row = tile_row;
        new_tile_col = east_tile;
    elseif (west_dir_val <= south_dir_val && west_dir_val <= east_dir_val && west_dir_val < north_dir_val)
        new_heading = 180;
        new_tile_row = tile_row;
        new_tile_col = west_tile;
    end
end

function center_rover_before_turn(loc_x, loc_y, s_cmd, s_rply, heading)
    speed = 1.5;
    if (loc_x <= 4 && loc_x >= 1)
        if (loc_x == 4 && heading == 180)
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (loc_x <= 8 && loc_x > 4)
        if ((heading == 0 && (loc_x == 5 || loc_x == 6)) || (heading == 180 && loc_x == 8))
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif ((heading == 0 && loc_x == 8) || (heading == 180 && loc_x == 5))
            cmdstring = [strcat('a1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (loc_x <= 12 && loc_x > 8)
        if ((heading == 0 && loc_x == 9) || (heading == 180 && loc_x == 12))
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif ((heading == 0 && loc_x == 12) || (heading == 180 && loc_x == 7))
            cmdstring = [strcat('a1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (loc_x <= 16 && loc_x > 12)
        if ((heading == 0 && loc_x == 13) || (heading == 180 && loc_x == 16))
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif ((heading == 0 && loc_x == 16) || (heading == 180 && loc_x == 13))
            cmdstring = [strcat('a1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (loc_x <= 20 && loc_x > 16)
        if ((heading == 0 && loc_x == 17) || (heading == 180 && loc_x == 20))
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif ((heading == 0 && loc_x == 20) || (heading == 180 && loc_x == 17))
            cmdstring = [strcat('a1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (loc_x <= 24 && loc_x > 20)
        if ((heading == 0 && (loc_x == 21 || loc_x == 22)) || (heading == 180 && loc_x == 24))
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif ((heading == 0 && loc_x == 24) || (heading == 180 && loc_x == 21))
            cmdstring = [strcat('a1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (loc_x <= 28 && loc_x > 24)
        if ((heading == 0 && loc_x == 25) || (heading == 180 && loc_x == 28))
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif ((heading == 0 && loc_x == 28) || (heading == 180 && loc_x == 25))
            cmdstring = [strcat('a1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (loc_x <= 32 && loc_x > 28)
        if ((heading == 0 && (loc_x == 29 || loc_x == 30)) || (heading == 180 && loc_x == 32))
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif ((heading == 0 && loc_x == 32) || (heading == 180 && loc_x == 29))
            cmdstring = [strcat('a1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    end
    
    if (loc_y <= 4 && loc_y >= 1)
        if (loc_y == 4 && heading == 90)
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (loc_y == 4 && heading == 270)
            cmdstring = [strcat('a1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (loc_y <= 8 && loc_y > 4)
        if ((heading == 270 && loc_y == 5) || (heading == 90 && loc_y == 8))
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif ((heading == 270 && loc_y == 8) || (heading == 90 && loc_y == 5))
            cmdstring = [strcat('a1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (loc_y <= 12 && loc_y > 8)
        if ((heading == 270 && loc_y == 9) || (heading == 90 && loc_y == 12))
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif ((heading == 270 && loc_y == 12) || (heading == 90 && loc_y == 7))
            cmdstring = [strcat('a1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (loc_y <= 16 && loc_y > 12)
        if ((heading == 270 && loc_y == 13) || (heading == 90 && loc_y == 16))
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif ((heading == 270 && loc_y == 16) || (heading == 90 && loc_y == 13))
            cmdstring = [strcat('a1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    end
    
    % Check if rover is in drop-off area, make sure centered
    
end

function new_heading = drive_to_new_heading(u, s_cmd, s_rply, heading, new_heading, loc_x, loc_y, dropoff_id, tile_x, tile_y, lz)
    dest_x1 = -1;
    dest_y1 = -1;
    dest_x2 = -1;
    dest_y2 = -1;
    dest_x = -1;
    dest_y = -1;
    if (lz == 1)
        dest_x1 = 3;
        dest_y1 = 1;
        dest_x2 = 1;
        dest_y2 = 3;
    elseif (dropoff_id == 1)
        dest_x = 3;
        dest_y = 3;
    elseif (dropoff_id == 2)
        dest_x = 1;
        dest_y = 6;
    elseif (dropoff_id == 3)
        dest_x = 1;
        dest_y = 8;
    elseif (dropoff_id == 4)
        dest_x = 4;
        dest_y = 8;
    end
    if (heading - new_heading == 0)
        % drive forward
        speed = u(1) / 2;
        if (speed > 6)
            speed = 6;
        end
        if (lz == 0 && (tile_x <= dest_x + 1 && tile_x >= dest_x - 1 && tile_y <= dest_y + 1 && tile_y >= dest_y - 1))
            speed = 3;
        elseif (lz == 1 && (tile_x <= dest_x1 + 1 && tile_x >= dest_x1 - 1 && tile_y <= dest_y1 + 1 && tile_y >= dest_y1 - 1) || (tile_x <= dest_x2 + 1 && tile_x >= dest_x2 - 1 && tile_y <= dest_y2 + 1 && tile_y >= dest_y2 - 1))
            speed = 3;
        end
        % Move rover forward one step
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp(['Move forward ' num2str(speed) ' inches'])
        new_heading = heading;
    elseif (heading - new_heading == -90 || heading - new_heading == 270)
        % drive left
        % Move left, first determine rover speed
        speed = u(2) / 2;
        if (speed > 6)
            speed = 6;
        end
        if (lz == 0 && (tile_x <= dest_x + 1 && tile_x >= dest_x - 1 && tile_y <= dest_y + 1 && tile_y >= dest_y - 1))
            speed = 3;
        end
        center_rover_before_turn(loc_x, loc_y, s_cmd, s_rply, heading)
        % Rotate rover 90 CW and move forward one step
        cmdstring = [strcat('r1-',num2str(90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp(['Move left ' num2str(speed) ' inches'])
        new_heading = mod(heading + 90,360);
    elseif (heading - new_heading == 90 || heading - new_heading == -270)
        % drive right
        % Move right, first determine rover speed
        if (u(4) < u(5))
            u_right = u(4);
        else
            u_right = u(5);
        end
        speed = u_right / 2;
        if (speed > 6)
            speed = 6;
        end
        if (lz == 0 && (tile_x <= dest_x + 1 && tile_x >= dest_x - 1 && tile_y <= dest_y + 1 && tile_y >= dest_y - 1))
            speed = 3;
        end
        center_rover_before_turn(loc_x, loc_y, s_cmd, s_rply, heading)
        % Rotate rover 90 CCW and move forward one step
        cmdstring = [strcat('r1-',num2str(-90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp(['Move right ' num2str(speed) ' inches'])
        new_heading = mod(heading - 90,360);
    elseif (abs(heading - new_heading) == 180)
        % drive reverse
        speed = u(3) / 2;
        if (speed > 6)
            speed = 6;
        end
        if (lz == 0 && (tile_x <= dest_x + 1 && tile_x >= dest_x - 1 && tile_y <= dest_y + 1 && tile_y >= dest_y - 1))
            speed = 3;
        end
        % Rotate rover 180 CW and move forward one step
        cmdstring = [strcat('r1-',num2str(180)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp(['Reverse ' num2str(speed) ' inches'])
        new_heading = mod(heading + 180,360);
    end
end

function [current_heading, p, k, M, ultra] = drive_to_destination(s_cmd, s_rply, current_heading, nav_grid, loc_x, loc_y, u_loc, rover_radius, M, p, ultra, k, ultrasonic_margin, dropoff_id, lz)
    % Direction value of 90 means north, 0 means east, 270 means south, 180
    % means west
    [tile_row, tile_col] = determine_rover_tile_loc(loc_x, loc_y);
    tile_val = nav_grid(tile_row, tile_col);
    rover_centered = 0;
    center_attempts = 0;
    unique_loc = 0;
    straighten_attempts = 0;
    first_unique_loc = 0;
    
    [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
    disp('Ultrasonic Measurements')
    disp(u)
    [tile_row, tile_col] = determine_rover_tile_loc(loc_x, loc_y);
    [desired_heading, new_tile_row, new_tile_col] = determine_new_heading(nav_grid, tile_row, tile_col);
    old_heading = current_heading;
    disp('### Heading towards destination ###')
    disp(['## Current Tile Location: (' num2str(tile_row) ',' num2str(tile_col) ') Current Tile Value: ' num2str(tile_val) '##'])
    disp(['## Next Tile Location: (' num2str(new_tile_row) ',' num2str(new_tile_col) ') Next Tile Value: ' num2str(nav_grid(new_tile_row, new_tile_col)) ' ##'])
    disp('###---------------------------###')
    
    while (tile_val > 0)
        disp(['########################## Step Count: ' num2str(k) ' ##########################'])
        
        unique_loc = 0;
        if (u_real(1) > 10 && u_real(2) > 10 && u_real(3) > 10 && (u_real(4) > 10 || u_real(5) > 10))
            unique_loc = 1;
            if (first_unique_loc == 0)
                first_unique_loc = 1;
            else
                first_unique_loc = 0;
            end
            [p,k,loc_y,loc_x, localized] = update_rover_location(p, M, current_heading, k);
            p = update_localization_map(u, M, p, ultra, k);
            [tile_row, tile_col] = determine_rover_tile_loc(loc_x, loc_y);
            tile_val = nav_grid(tile_row, tile_col);
            [desired_heading, new_tile_row, new_tile_col] = determine_new_heading(nav_grid, tile_row, tile_col);
            disp('### In Unique Location ###')
            disp(['## Current Tile Location: (' num2str(tile_row) ',' num2str(tile_col) ') Current Tile Value: ' num2str(tile_val) '##'])
            disp(['## Next Tile Location: (' num2str(new_tile_row) ',' num2str(new_tile_col) ') Next Tile Value: ' num2str(nav_grid(new_tile_row, new_tile_col)) ' ##'])
            disp('###---------------------------###')
        end
        
        if (unique_loc == 0 && straighten_attempts < 5 && old_heading ~= current_heading)
            rover_straight = straighten_rover(u, s_cmd, s_rply, unique_loc);
            if (rover_straight == 0)
                straighten_attempts = straighten_attempts + 1;
            else
                straighten_attempts = 0;
            end
        else
            straighten_attempts = 0;
            rover_straight = 1;
        end

        if (rover_straight == 1 && center_attempts < 2)
            rover_centered = center_rover(u, s_cmd, s_rply, ultrasonic_margin, rover_radius, unique_loc, first_unique_loc, current_heading);
            if (first_unique_loc == 1)
                rover_centered = 0;
                unique_loc = 0;
            end
            if (rover_centered == 0)
                center_attempts = center_attempts + 1;
            else
                center_attempts = 0;
            end
        else
            rover_centered = 1;
            center_attempts = 0;
        end
        if ((rover_centered == 1 && rover_straight == 1) || unique_loc == 1)
            old_heading = current_heading;
            current_heading = drive_to_new_heading(u, s_cmd, s_rply, current_heading, desired_heading, loc_x, loc_y, dropoff_id, tile_row, tile_col, lz);
        end
        
        [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
        disp('Ultrasonic Measurements')
        disp(u)
        
        if (rover_straight == 1)
            [p,k,loc_y,loc_x, localized] = update_rover_location(p, M, current_heading, k);
            p = update_localization_map(u, M, p, ultra, k);
            [tile_row, tile_col] = determine_rover_tile_loc(loc_x, loc_y);
            tile_val = nav_grid(tile_row, tile_col);
            [desired_heading, new_tile_row, new_tile_col] = determine_new_heading(nav_grid, tile_row, tile_col);
            disp('### After Centering or Moving ###')
            disp(['## Current Tile Location: (' num2str(tile_row) ',' num2str(tile_col) ') Current Tile Value: ' num2str(tile_val) '##'])
            disp(['## Next Tile Location: (' num2str(new_tile_row) ',' num2str(new_tile_col) ') Next Tile Value: ' num2str(nav_grid(new_tile_row, new_tile_col)) ' ##'])
            disp('###---------------------------###')
        end
    end
end

function [heading, p, k, M, ultra, loc_x, loc_y] = find_and_load_block(s_cmd, s_rply, rover_radius, u_loc, rover_dist_thresh, p, k, M, ultra, heading, ultrasonic_margin)
    [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
    deg = 10;
    tot_rot = 0;
    detect_thresh_max = 2.1;
    detect_thresh_min = 1.4;
    prox_thresh = 6.5;
    found_block = 0;
    init_heading = heading;
    u4_u5_diff = abs(u(4) - u(5));
    u_right = u(4);
    if (u(5) > u(4))
        u_right = u(5);
    end
    
    if (u4_u5_diff > 5)
        disp('Rover is next to a corner, move forward 4 inches before searching for block')
        speed = 4;
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
        disp('Ultrasonic Sensor')
        disp(u)
        [p,k,loc_y,loc_x, localized] = update_rover_location(p, M, init_heading, k);
        p = update_localization_map(u, M, p, ultra, k);
    end
    
    if (u(2) < u_right)
        while (found_block == 0)
            rot = -1*deg;
            cmdstring = [strcat('r1-',num2str(rot)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            tot_rot = tot_rot + rot;
            heading = heading + rot;
            [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
            disp('Ultrasonic Sensor')
            disp(u)
            disp(["tot_rot: " tot_rot]);
            if (u(1)/u(6) > detect_thresh_min && u(1)/u(6) < detect_thresh_max && (abs(tot_rot) > 70 || abs(tot_rot) < 40))
                found_block = 1;
                disp("Found block!");
            end
        end
    else
        while (found_block == 0)
            rot = deg;
            cmdstring = [strcat('r1-',num2str(rot)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            tot_rot = tot_rot + rot;
            heading = heading + rot;
            [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
            disp('Ultrasonic Sensor')
            disp(u)
            disp(["tot_rot: " tot_rot]);
            if (u(1)/u(6) > detect_thresh_min && u(1)/u(6) < detect_thresh_max && (abs(tot_rot) > 70 || abs(tot_rot) < 40))
                found_block = 1;
                disp("Found block!");
            end                
        end
    end
    
    speed = u(6) - prox_thresh;
    speed_dir = speed / abs(speed);
    disp(['Giving gripper enough space to open, moving: ' num2str(speed) ' inches'])
    cmdstring = [strcat('d1-',num2str(speed)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
    disp('Ultrasonic Sensor')
    disp(u)
    [p,k,loc_y,loc_x, localized] = update_rover_location(p, M, speed_dir*heading, k);
    p = update_localization_map(u, M, p, ultra, k);
    cmdstring = [strcat('g1-',num2str(180)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    speed = 3;
    disp(['Move towards the block, and pick it up. Move: ' num2str(speed) ' inches'])
    cmdstring = [strcat('d1-',num2str(prox_thresh)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
    disp('Ultrasonic Sensor')
    disp(u)
    [p,k,loc_y,loc_x, localized, max_prob] = update_rover_location(p, M, heading, k);
    p = update_localization_map(u, M, p, ultra, k);
    cmdstring = [strcat('g1-',num2str(40)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    disp("############## Block Loaded!! ##############");
    % center rover
    cmdstring = [strcat('r1-',num2str(-1*tot_rot)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    heading = heading - tot_rot;
    [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
    rover_centered = 0;
    unique_loc = 0;
    first_unique_loc = 0;
    while (rover_centered == 0)
        rover_centered = center_rover(u, s_cmd, s_rply, ultrasonic_margin, rover_radius, unique_loc, first_unique_loc, heading);
        [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
        [p,k,loc_y,loc_x, localized, max_prob] = update_rover_location(p, M, heading, k);
        p = update_localization_map(u, M, p, ultra, k);
    end
    
    while (u(1) > rover_dist_thresh*1.25)
        speed = u(1) / 2;
        if (speed > 3)
            speed = 3;
        end
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
       [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
        [p,k,loc_y,loc_x, localized, max_prob] = update_rover_location(p, M, heading, k);
        p = update_localization_map(u, M, p, ultra, k);
    end
    
    disp("Rover is ready to move to the drop off zone!");
end

function deliver_block_and_close_gate(s_cmd, s_rply, rover_radius, u_loc, ultrasonic_margin, heading)
    disp("Make sure rover is safe distance from walls before unloading")
    [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
    gripper_close_thresh = 6.5;
    rover_centered = 0;
    first_unique_loc = 0;
    while (rover_centered == 0)
        unique_loc = 0;
        if (u_real(1) > 10 && u_real(2) > 10 && u_real(3) > 10 && (u_real(4) > 10 || u_real(5) > 10))
            unique_loc = 1;
            if (first_unique_loc == 0)
                first_unique_loc = 1;
            else
                first_unique_loc = 0;
            end
            disp('unique_loc')
        end
        rover_centered = center_rover(u, s_cmd, s_rply, ultrasonic_margin, rover_radius, unique_loc, first_unique_loc, heading);
        [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
    end
    
    if (u(1) <= gripper_close_thresh)
        while (u(1) <= gripper_close_thresh)
            speed = (gripper_close_thresh - u(1));
            if (speed > 3)
                speed = 3;
            end
            disp(['Rover reverse ' num2str(speed) ' inches'])
            cmdstring = [strcat('a1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
        end
    else
        while (u(1) > gripper_close_thresh)
            speed = (u(1) - gripper_close_thresh);
            if (speed > 3)
                speed = 3;
            end
            disp(['Rover move forward ' num2str(speed) ' inches'])
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
        end
    end
        
    disp("Unloading block!");
    cmdstring = [strcat('g1-',num2str(180)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    speed = 6.5;
    disp('Moving rover back 6 inches to close gate')
    cmdstring = [strcat('a1-',num2str(speed)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    disp('Close gripper gate')
    cmdstring = [strcat('g1-',num2str(40)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
end