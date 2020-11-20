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
u_loc = [3.09,-1.56 ; 1.13,3.52 ; -3.49,1.04 ; -1.15,-3.43 ; 1.08,-3.43 ; 2.68,3.52];
pos = [0,0,0];  % Position (x,y,rotation)
rot_stuck = 90;
stepcount = 0;

rover_radius = 4.915;
ultrasonic_margin = 0.18;
drive_margin = 0;
cmdstring_history = ["a","b","c"];
cmdstring_history_idx = 1;

% Stuck condition variables
log = 1;
stuck_cond = 0.05;
u_mat = zeros(3,6);

%% Main Rover Control Code
rover_centered = 0;
rover_unique_loc = 0;
while 1    
    % Rotate rover until sensor measurements from u4/u5 AND u1/u6 are
    % within an acceptable range
    u = take_ultrasonic_measurements(s_cmd, s_rply);
    disp('Ultrasonic; Before center, before straighten, after move')
    disp(u)
    rover_centered = center_rover(u, u_loc, s_cmd, s_rply, ultrasonic_margin, rover_radius);

    if (rover_centered == 1)
        % Move the rover one step in the optimal direction
        u = take_ultrasonic_measurements(s_cmd, s_rply);
        disp('Ultrasonic; After straighten, after center, before move')
        disp(u)
        move_rover(u, u_loc, s_cmd, s_rply, ultrasonic_margin, rover_radius)
    end
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

function [rover_straight, cmd_history] = straighten_rover(u, u_loc, s_cmd, s_rply, ultrasonic_margin, cmdstring_history, cmd_history_idx, rover_radius)
    % This function ensures that after each step of the rover, it is
    % continuing to move straight
    u1_ = u(1) - (rover_radius - abs(u_loc(1,1)));
    u2_ = u(2) - (rover_radius - abs(u_loc(2,2)));
    u4_ = u(4) - (rover_radius - abs(u_loc(4,2)));
    u5_ = u(5) - (rover_radius - abs(u_loc(5,2)));
    u6_ = u(6) - (rover_radius - abs(u_loc(6,1)));
    disp('u1,u6,u4,u5,u2')
    disp([u1_,u6_,u4_,u5_, u2_])
    leftright_thresh_straight = 2.17;
    leftright_thresh_straight_min = leftright_thresh_straight - leftright_thresh_straight * ultrasonic_margin;
    leftright_thresh_straight_max = leftright_thresh_straight + leftright_thresh_straight * ultrasonic_margin;
    lower_thresh_straight = 0.35;
    upper_thresh_straight = 5;
    cmd_history = cmdstring_history;
    u4_u5_diff = abs(u4_ - u5_);
    if (u4_ < u5_)
        u45_ = u4_;
    elseif (u5_ < u4_)
        u45_ = u5_;
    end
    u2_u45_diff = abs(u2_ - u45_)
    %u1_u6_diff = 0;%abs(u1 - u6);
    min_rot = 1;
    min_rot_thresh = (upper_thresh_straight - lower_thresh_straight)*0.15 + lower_thresh_straight;
    med_rot = 5;
    max_rot = 10;
    max_rot_thresh = (upper_thresh_straight - lower_thresh_straight)*0.65 + lower_thresh_straight;
    
    %rot = min_rot;
    
    if ((u2_u45_diff < leftright_thresh_straight_min) || (u2_u45_diff > leftright_thresh_straight_max))
        % Determine number of degrees to turn rover
        if (u2_u45_diff > max_rot_thresh)
            rot = max_rot;
        elseif (u2_u45_diff <= max_rot_thresh && u2_u45_diff > min_rot_thresh)
            rot = med_rot;
        elseif (u2_u45_diff <= min_rot_thresh)
            rot = min_rot;
        end
        % Determine if to turn rover CW or CCW
        if (u4_ > (u5_))
            cmd = [strcat('r1--',num2str(rot)) newline];
            reply_ = tcpclient_write(cmd, s_cmd, s_rply);
        elseif (u4_ <= (u5_))
            cmd = [strcat('r1-',num2str(rot)) newline];
            reply_ = tcpclient_write(cmd, s_cmd, s_rply);
        end
        disp('Straighten rover')
    end
    
    if ((u2_u45_diff < leftright_thresh_straight || u2_u45_diff > upper_thresh_straight) && (u4_u5_diff < lower_thresh_straight || u4_u5_diff > upper_thresh_straight))
        rover_straight = 1;
        disp('rover_straight')
        disp(rover_straight)
    else
        rover_straight = 0;
        disp('rover_straight')
        disp(rover_straight)
    end
end

function rover_centered = center_rover(u, u_loc, s_cmd, s_rply, ultrasonic_margin, rover_radius)
    % This function centers the rover as much as possible to ensure
    % straight motion
    u1 = u(1) - (rover_radius - abs(u_loc(1,1)));
    u2 = u(2) - (rover_radius - abs(u_loc(2,2)));
    u3 = u(3) - (rover_radius - abs(u_loc(3,1)));
    u4 = u(4) - (rover_radius - abs(u_loc(4,2)));
    u5 = u(5) - (rover_radius - abs(u_loc(5,2)));
    u1_max_dist = 2.91 + 2.91*ultrasonic_margin;
    u_left = u2;
    if (u4 < u5)
        u_right = u4;
    else
        u_right = u5;
    end
    disp('u_left,u_right')
    disp([u_left,u_right])
    centered_dist_from_wall = 6 - rover_radius;
    centered_dist_from_wall_min = centered_dist_from_wall - centered_dist_from_wall * ultrasonic_margin;
    centered_dist_from_wall_max = centered_dist_from_wall + centered_dist_from_wall * ultrasonic_margin;
    unique_loc = 0;
    if (u_left > 10 && u_right > 10 && u1 > 10 && u3 > 10)
        unique_loc = 1;
        disp('unique_loc')
        disp(unique_loc)
    end
        
    if (unique_loc == 0 && ((u_left > centered_dist_from_wall_min && u_left < centered_dist_from_wall_max) || (u_right > centered_dist_from_wall_min && u_right < centered_dist_from_wall_max)))
        rover_centered = 1;
        disp('rover_centered')
        disp(rover_centered)
    elseif (unique_loc == 0 && (u_left < centered_dist_from_wall_min || (u_right < u_left && u_right > centered_dist_from_wall_max)))
        % Either left side is too close or right side is too far, move right
        if (u1 > u1_max_dist)
            speed = u_right / 2;        
            cmdstring = [strcat('r1-',num2str(-22)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('r1-',num2str(22)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            disp('Left/Right side too close/far, move right. r1-22, d1-1, r1--22')
            rover_centered = 0;
        else
            rover_centered = 1;
            disp('rover_centered')
            disp(rover_centered)
        end
    elseif (unique_loc == 0 && (u_right < centered_dist_from_wall_min || (u_left < u_right && u_left > centered_dist_from_wall_max)))
        % Eithert right side is too close or left side is too far, move left
        if (u1 > u1_max_dist)
            speed = u_left / 2;        
            cmdstring = [strcat('r1-',num2str(22)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('r1-',num2str(-22)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            disp('Right/Left side too close/far, move left. r1--22, d1-1, r1-22')
            rover_centered = 0;
        else
            rover_centered = 1;
            disp('rover_centered')
            disp(rover_centered)
        end
    elseif (unique_loc == 1)
        rover_centered = 1;
        disp('rover_centered')
        disp(rover_centered)
    end
end

function move_rover(u, u_loc, s_cmd, s_rply, ultrasonic_margin, rover_radius)
    % This function's purpose is to move the rover one step in the optimal
    % direction
    u1 = u(1) - (rover_radius - abs(u_loc(1,1)));
    u2 = u(2) - (rover_radius - abs(u_loc(2,2)));
    u3 = u(3) - (rover_radius - abs(u_loc(3,1)));
    u4 = u(4) - (rover_radius - abs(u_loc(4,2)));
    u5 = u(5) - (rover_radius - abs(u_loc(5,2)));
    u6 = u(6) - (rover_radius - abs(u_loc(6,1)));
    disp('u1,u2,u3,u4,u5,u6')
    disp([u1,u2,u3,u4,u5,u6])
    u1_max_dist = 2.91 + 2.91*ultrasonic_margin;
    u2_max_dist = 2.48 + 2.48*ultrasonic_margin;
    u3_max_dist = 2.51 + 2.51*ultrasonic_margin;
    u45_max_dist = 2.57 + 2.57*ultrasonic_margin;
    u45 = abs(u4 + u5)/2;
        
    if (u2 > u45 && u2 > u2_max_dist && u1 <= u1_max_dist)
        % Move left, first determine rover speed
        speed = u2 / 2;
        % Rotate rover 90 CW and move forward one step
        cmdstring = [strcat('r1-',num2str(90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Move left')
        disp(cmdstring)
    elseif (u45 >= u2 && u45 >= u45_max_dist && u1 <= u1_max_dist)
        % Move right, first determine rover speed
        if (u4 <= u5)
            speed = u4 / 2;
        elseif (u5 < u4)
            speed = u5 / 2;
        end
        % Rotate rover 90 CCW and move forward one step
        cmdstring = [strcat('r1-',num2str(-90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Move right')
        disp(cmdstring)
    elseif (u1 > u1_max_dist)
        % Move forward, first determine rover speed
        speed = u1 / 2;
        % Move rover forward one step
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Move forward')
        disp(cmdstring)
    elseif (u3 > u3_max_dist)
        % Move backwards, first determine rover speed
        speed = u3 /2;
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