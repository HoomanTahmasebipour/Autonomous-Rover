%% Rover Initialization and Setup
clear
close all
clc

sim = 1;

if sim
% Initialize tcp server to read and respond to algorithm commands
[s_cmd, s_rply] = tcp_setup();
fopen(s_cmd);
%fopen(s_rply);
else
    %connect to rover Bluetooth
end

% Robot Sensor Measurements
u = [0,0,0,0,0,0];  % Ultrasonic measurements
pos = [0,0,0];  % Position (x,y,rotation)
rot_stuck = 90;
stepcount = 0;

ultrasonic_margin = 0.05;
drive_margin = 0;
cmdstring_history = ['','',''];
cmdstring_history_idx = 1;

% Stuck condition variables
log = 1;
stuck_cond = 0.05;
u_mat = zeros(3,6);

%% Ensure proper rover orientation
% The purpose of this section is to continously rotate the rover until it is pointing as
% straight as possible
u = take_ultrasonic_measurements(s_cmd, s_rply);
while u(1) < u1_max_dist + 12
    u = take_ultrasonic_measurements(s_cmd, s_rply);
    %Rotate rover CW until driving straight
    cmdstring = [strcat('r1-',num2str(10)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
end

%% Main Rover Control Code
rover_centered = 0;
while 1
    disp('rover_straight')
    disp(rover_straight)
    disp('rover_centered')
    disp(rover_centered)
    
    % Rotate rover until sensor measurements from u4/u5 AND u1/u6 are
    % within an acceptable range
    u = take_ultrasonic_measurements(s_cmd, s_rply);
    disp('Ultrasonic; Before move, before straighten, after move')
    disp(u)
    return_val = straighten_rover(u, s_cmd, s_rply, drive_margin, cmdstring_history, cmdstring_history_idx);
    rover_straight = return_val(1);
    cmdstring_history = return_val(2);
    if (mod(cmdstring_history_idx,3) == 0)
        cmdstring_history_idx = 1;
    else
        cmdstring_history_idx = cmdstring_history_idx + 1;
    end
    if (rover_straight == 1) 
        % Make sure the rover is centered in a tile, wihtin an acceptable
        % margin
        u = take_ultrasonic_measurements(s_cmd, s_rply);
        disp('Ultrasonic; After straighten, before center, before move')
        disp(u)
        rover_centered = center_rover(u, s_cmd, s_rply, ultrasonic_margin);
    end
    if (rover_straight == 1 && rover_centered == 1)
        % Move the rover one step in the optimal direction
        u = take_ultrasonic_measurements(s_cmd, s_rply);
        disp('Ultrasonic; After straighten, after center, before move')
        disp(u)
        move_rover(u, s_cmd, s_rply, ultrasonic_margin);
    end
    % If the rover has been stuck in the same position for three commands, try to get out
end

%% Helper functions
function [u] = take_ultrasonic_measurements(s_cmd, s_rply)
    % This function takes ultrasonic readings as many as num_measurements,
    % averages them out, and returns them in a 1x6 vector.
    u = [0,0,0,0,0,0];
    num_measurements = 1;
    
    for i = 1:num_measurements
        for ct = 1:6
            cmdstring = [strcat('u',num2str(ct)) newline];
            u(ct) = u(ct) + tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    end
    % average out the sensor values
    u = u / num_measurements;
end

function rover_straight_, cmdstring_history = straighten_rover(u, s_cmd, s_rply, drive_margin, cmdstring_history)
    % This function ensures that after each step of the rover, it is
    % continuing to move straight
    lower_thresh_straight = 0.5;
    upper_thresh_straight = 5;
    
    u4_u5_diff = abs(u(4) - u(5));
    u1_u6_diff = abs(u(1) - u(6));
    min_rot = 1;
    min_rot_thresh = (upper_thresh_straight - lower_thresh_straight)*0.10 + lower_thresh_straight;
    med_rot = 5;
    max_rot = 10;
    max_rot_thresh = (upper_thresh_straight - lower_thresh_straight)*0.5 + lower_thresh_straight;
    
    rot = min_rot;
    
    if ((u4_u5_diff > lower_thresh_straight && u4_u5_diff < upper_thresh_straight) || (u1_u6_diff > lower_thresh_straight && u1_u6_diff < upper_thresh_straight))
        if (u4_u5_diff > u1_u6_diff)
            % Determine number of degrees to turn rover
            if (u4_u5_diff > max_rot_thresh)
                rot = max_rot;
            elseif (u4_u5_diff <= max_rot_thresh && u4_u5_diff > min_rot_thresh)
                rot = med_rot;
            elseif (u4_u5_diff <= min_rot_thresh)
                rot = min_rot;
            end
            % Determine if to turn rover CW or CCW
            if (u(4) > (u(5) + u(5) * drive_margin))
                cmdstring = [strcat('r1-',num2str(rot)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            elseif (u(4) <= (u(5) - u(5) * drive_margin))
                cmdstring = [strcat('r1--',num2str(rot)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            end
            disp('Straighten rover based on u4/u5')
        elseif (u1_u6_diff >= u4_u5_diff)
            % Determine number of degrees to turn rover
            if (u1_u6_diff > max_rot_thresh)
                rot = max_rot;
            elseif (u1_u6_diff <= max_rot_thresh && u4_u5_diff > min_rot_thresh)
                rot = med_rot;
            elseif (u1_u6_diff <= min_rot_thresh)
                rot = min_rot;
            end
            % Determine if to turn rover CW or CCW
            disp('rot')
            disp(rot)
            if (u(1) > (u(6) + u(6) * drive_margin))
                cmdstring = [strcat('r1-',num2str(rot)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            elseif (u(1) <= (u(6) - u(6) * drive_margin))
                cmdstring = [strcat('r1--',num2str(rot)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            end
            disp('Straighten rover based on u1/u6')
            disp(cmdstring)
        end
    end
    
    if ((u4_u5_diff < lower_thresh_straight || u4_u5_diff > upper_thresh_straight) && (u1_u6_diff < lower_thresh_straight || u1_u6_diff > upper_thresh_straight))
        rover_straight_ = 1;
    else
        rover_straight_ = 0;
    end
    
    cmdstring_history(cmdstring_history_idx) = cmdstring;
end

function rover_centered = center_rover(u, s_cmd, s_rply, ultrasonic_margin)
    % This function centers the rover as much as possible to ensure
    % straight motion
    u2_max_dist = 2.48 + 2.48 * ultrasonic_margin;
    u45_max_dist = 2.57 + 2.57 * ultrasonic_margin;
    u2_min_dist = 2.48 - 2.48 * ultrasonic_margin;
    u45_min_dist = 2.57 - 2.57 * ultrasonic_margin;
    u45 = abs(u(4) + u(5))/2;
    speed = 0.2;
    
    if (u(2) < u2_min_dist || (u(2) < u45 && u(2) > u2_max_dist))
        % Either left side is too close or its too far, move right
        cmdstring = [strcat('r1-',num2str(90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('r1-',num2str(-90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Left side too close/far. r1-90, d1-1, r1--90')
        rover_centered = 0;
    elseif (u(2) < u45_min_dist || (u45 < u(2) && u45 > u45_max_dist))
        % Either right side is too close or its too far, move left
        cmdstring = [strcat('r1-',num2str(-90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('r1-',num2str(90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Right side too close/far. r1--90, d1-1, r1-90')
        rover_centered = 0;
    else
        rover_centered = 1;
    end
end

function move_rover(u, s_cmd, s_rply, ultrasonic_margin)
    % This function's purpose is to move the rover one step in the optimal
    % direction
    u1_max_dist = 2.91 + 2.91 * ultrasonic_margin;
    u2_max_dist = 2.48 + 2.48 * ultrasonic_margin;
    u3_max_dist = 2.51 + 2.51 * ultrasonic_margin;
    u45_max_dist = 2.57 + 2.57 * ultrasonic_margin;
    u45 = abs(u(4) + u(5))/2;
    u16 = abs(u(1) + u(6))/2;
    
    min_speed = 0.2;
    min_speed_dist_thresh = 5;
    med_speed = 2;
    max_speed_dist_thresh = 10;
    max_speed = 3;
    
    if (u(2) > u45 && u(2) > u2_max_dist && u16 < u1_max_dist)
        % Move left, first determine rover speed
        if (u(2) < min_speed_dist_thresh)
            speed = min_speed;
        elseif (u(2) > min_speed_dist_thresh && u(2) < max_speed_dist_thresh)
            speed = med_speed;
        elseif (u(2) > max_speed_dist_thresh)
            speed = max_speed;
        end
        % Rotate rover 90 CW and move forward one step
        cmdstring = [strcat('r1-',num2str(90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Move left')
        disp(cmdstring)
    elseif (u45 > u(2) && u45 > u45_max_dist && u16 < u1_max_dist)
        % Move right, first determine rover speed
        if (u45 < min_speed_dist_thresh)
            speed = min_speed;
        elseif (u45 > min_speed_dist_thresh && u45 < max_speed_dist_thresh)
            speed = med_speed;
        elseif (u45 > max_speed_dist_thresh)
            speed = max_speed;
        end
        % Rotate rover 90 CCW and move forward one step
        cmdstring = [strcat('r1-',num2str(-90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Move right')
        disp(cmdstring)
    elseif (u16 > u1_max_dist)
        % Move forward, first determine rover speed
        if (u16 < min_speed_dist_thresh)
            speed = min_speed;
        elseif (u16 > min_speed_dist_thresh && u16 < max_speed_dist_thresh)
            speed = med_speed;
        elseif (u16 > max_speed_dist_thresh)
            speed = max_speed;
        end
        % Move rover forward one step
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Move forward')
        disp(cmdstring)
    elseif (u(3) > u3_max_dist)
        % Move backwards, first determine rover speed
        if (u(3) < min_speed_dist_thresh)
            speed = min_speed;
        elseif (u(3) > min_speed_dist_thresh && u(3) < max_speed_dist_thresh)
            speed = med_speed;
        elseif (u(3) > max_speed_dist_thresh)
            speed = max_speed;
        end
        % Rotate rover 180 CW and move forward one step
        cmdstring = [strcat('r1-',num2str(90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Reverse')
        disp(cmdstring)
    end
end