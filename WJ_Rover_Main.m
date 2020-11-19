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
ultrasonic_margin = (6 - rover_radius) / 6;
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
    disp('Ultrasonic; Before straighten, before center, after move')
    disp(u)
    [rover_straight, cmdstring_history] = straighten_rover(u, u_loc, s_cmd, s_rply, drive_margin, cmdstring_history, cmdstring_history_idx, rover_radius);
    if (rover_straight == 1) 
        % Make sure the rover is centered in a tile, wihtin an acceptable
        % margin
        u = take_ultrasonic_measurements(s_cmd, s_rply);
        disp('Ultrasonic; After straighten, before center, before move')
        disp(u)
        rover_centered = center_rover(u, u_loc, s_cmd, s_rply, ultrasonic_margin, rover_radius);
    end
    if (rover_straight == 1 && rover_centered == 1)
        % Move the rover one step in the optimal direction
        u = take_ultrasonic_measurements(s_cmd, s_rply);
        disp('Ultrasonic; After straighten, after center, before move')
        disp(u)
        move_rover(u, u_loc, s_cmd, s_rply, ultrasonic_margin, rover_radius)
    end
    % Handle the case that the rover is oscilating between commands
    if (mod(cmdstring_history_idx,3) == 0)
        cmdstring_history_idx = 1;
        if (cmdstring_history(1) == cmdstring_history(3))
            disp('oscilating')
            %is_r1 = strfind(cmdstring_history(1), "r1");
            %is_CCW = strfind(cmdstring_history(1), "--");
            %if (~(isempty(is_r1)) && ~(isempty(is_CCW)))
            %    cmdstring = [strcat('a1-',num2str(1)) newline];
            %    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            %    cmdstring = [strcat('r1-',num2str(5)) newline];
            %    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            %elseif (~(isempty(is_r1)))
            %    cmdstring = [strcat('a1-',num2str(1)) newline];
            %    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            %    cmdstring = [strcat('r1-',num2str(-5)) newline];
            %    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            %end
            cmdstring = [strcat('d1-',num2str(1)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    else
        cmdstring_history_idx = cmdstring_history_idx + 1;
    end
    % Specific logic for when the rover is in the 
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

function [rover_straight, cmd_history] = straighten_rover(u, u_loc, s_cmd, s_rply, drive_margin, cmdstring_history, cmd_history_idx, rover_radius)
    % u(1) is the front sensor ; u(2) is left ; u(3) is back ;
    % u(4) is right back ; u(5) is right front ; u(6) is gripper
    % This function ensures that after each step of the rover, it is
    % continuing to move straight
    u1 = u(1) - (rover_radius - abs(u_loc(1,1)));
    u4 = u(4) - (rover_radius - abs(u_loc(4,2)));
    u5 = u(5) - (rover_radius - abs(u_loc(5,2)));
    u6 = u(6) - (rover_radius - abs(u_loc(6,1)));
    disp('u1,u6,u4,u5')
    disp([u1,u6,u4,u5])
    lower_thresh_straight = 0.35;
    upper_thresh_straight = 5;
    cmd_history = cmdstring_history;
    u4_u5_diff = abs(u4 - u5);
    u1_u6_diff = 0;%abs(u1 - u6);
    min_rot = 1;
    min_rot_thresh = (upper_thresh_straight - lower_thresh_straight)*0.15 + lower_thresh_straight;
    med_rot = 5;
    max_rot = 10;
    max_rot_thresh = (upper_thresh_straight - lower_thresh_straight)*0.65 + lower_thresh_straight;
    
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
            if (u4 > (u5 + u5 * drive_margin))
                cmd = strcat('r1-',num2str(rot));
                cmdstring = [cmd newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            elseif (u4 <= (u5 - u5 * drive_margin))
                cmd = strcat('r1--',num2str(rot));
                cmdstring = [cmd newline];
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
            if (u1 > (u6 + u6 * drive_margin))
                cmd = strcat('r1--',num2str(rot));
                cmdstring = [cmd newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            elseif (u1 <= (u6 - u6 * drive_margin))
                cmd = strcat('r1-',num2str(rot));
                cmdstring = [cmd newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            end
            disp('Straighten rover based on u1/u6')
        end
        disp(cmdstring)
        cmd_history(cmd_history_idx) = cmd;
    end
    
    if ((u4_u5_diff < lower_thresh_straight || u4_u5_diff > upper_thresh_straight) && (u1_u6_diff < lower_thresh_straight || u1_u6_diff > upper_thresh_straight))
        rover_straight = 1;
    else
        rover_straight = 0;
    end
end

function rover_centered = center_rover(u, u_loc, s_cmd, s_rply, ultrasonic_margin, rover_radius)
    % This function centers the rover as much as possible to ensure
    % straight motion
    u2 = u(2) - (rover_radius - abs(u_loc(2,2)));
    u4 = u(4) - (rover_radius - abs(u_loc(4,2)));
    u5 = u(5) - (rover_radius - abs(u_loc(5,2)));
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
    if (u_left > 10 && u_right > 10)
        unique_loc = 1;
        centered_dist_from_wall_min = 17.5;%centered_dist_from_wall - centered_dist_from_wall * ultrasonic_margin;
        centered_dist_from_wall_max = 18.5;%centered_dist_from_wall + centered_dist_from_wall * ultrasonic_margin;
    end
        
    micro_speed = 0.2;
    micro_speed_dist_thresh = 3;
    min_speed = 5;
    min_speed_dist_thresh = 4;
    med_speed = 1;
    max_speed_dist_thresh = 7;
    max_speed = 2;
    
    if (unique_loc == 0 && ((u_left > centered_dist_from_wall_min && u_left < centered_dist_from_wall_max) || (u_right > centered_dist_from_wall_min && u_right < centered_dist_from_wall_max)))
        rover_centered = 1;
    elseif (unique_loc == 0 && (u_left < centered_dist_from_wall_min || (u_right < u_left && u_right > centered_dist_from_wall_max)))
        % Either left side is too close or right side is too far, move right
        if (u_right < micro_speed_dist_thresh)
            speed = micro_speed;
        elseif (u_right > micro_speed_dist_thresh && u_right <= min_speed_dist_thresh)
            speed = min_speed;
        elseif (u_right > min_speed_dist_thresh && u_right <= max_speed_dist_thresh)
            speed = med_speed;
        elseif (u_right > max_speed_dist_thresh)
            speed = max_speed;
        end
        
        cmdstring = [strcat('r1-',num2str(-90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('r1-',num2str(90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Left/Right side too close/far, move right. r1-90, d1-1, r1--90')
        rover_centered = 0;
    elseif (unique_loc == 0 && (u_right < centered_dist_from_wall_min || (u_left < u_right && u_left > centered_dist_from_wall_max)))
        % Either right side is too close or left side is too far, move left
        
        if (u_left < micro_speed_dist_thresh)
            speed = micro_speed;
        elseif (u_left > micro_speed_dist_thresh && u_left <= min_speed_dist_thresh)
            speed = min_speed;
        elseif (u_left > min_speed_dist_thresh && u_left <= max_speed_dist_thresh)
            speed = med_speed;
        elseif (u_left > max_speed_dist_thresh)
            speed = max_speed;
        end
        
        cmdstring = [strcat('r1-',num2str(90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('r1-',num2str(-90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Right/Left side too close/far, move left. r1--90, d1-1, r1-90')
        rover_centered = 0;
    elseif (unique_loc == 1)
        rover_centered = 1;
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
    u16 = abs(u1 + u6)/2;
    
    micro_speed = 0.5;
    micro_speed_dist_thresh = 3;
    min_speed = 1;
    min_speed_dist_thresh = 4;
    med_speed = 2;
    max_speed_dist_thresh = 7;
    max_speed = 3;
    super_speed_dist_thresh = 10;
    super_speed = 4;
    
    if (u2 > u45 && u2 > u2_max_dist && u16 <= u1_max_dist)
        % Move left, first determine rover speed
        if (u2 < micro_speed_dist_thresh)
            speed = micro_speed;
        elseif (u2 > micro_speed_dist_thresh && u2 <= min_speed_dist_thresh)
            speed = min_speed;
        elseif (u2 > min_speed_dist_thresh && u2 <= max_speed_dist_thresh)
            speed = med_speed;
        elseif (u2 > max_speed_dist_thresh && u2 <= super_speed_dist_thresh)
            speed = max_speed;
        elseif (u2 > super_speed_dist_thresh)
            speed = super_speed;
        end
        % Rotate rover 90 CW and move forward one step
        cmdstring = [strcat('r1-',num2str(90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Move left')
        disp(cmdstring)
    elseif (u45 >= u2 && u45 >= u45_max_dist && u16 <= u1_max_dist)
        % Move right, first determine rover speed
        if (u45 < micro_speed_dist_thresh)
            speed = micro_speed;
        elseif (u45 > micro_speed_dist_thresh && u45 <= min_speed_dist_thresh)
            speed = min_speed;
        elseif (u45 > min_speed_dist_thresh && u45 <= super_speed_dist_thresh)
            speed = max_speed;
        elseif (u45 > super_speed_dist_thresh)
            speed = super_speed;
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
        if (u16 < micro_speed_dist_thresh)
            speed = micro_speed;
        elseif (u16 > micro_speed_dist_thresh && u16 <= min_speed_dist_thresh)
            speed = min_speed;
        elseif (u16 > min_speed_dist_thresh && u16 <= max_speed_dist_thresh)
            speed = med_speed;
        elseif (u16 > max_speed_dist_thresh && u16 <= super_speed_dist_thresh)
            speed = max_speed;
        elseif (u16 > super_speed_dist_thresh)
            speed = super_speed;
        end
        % Move rover forward one step
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Move forward')
        disp(cmdstring)
    elseif (u3 > u3_max_dist)
        % Move backwards, first determine rover speed
        if (u3 < micro_speed_dist_thresh)
            speed = micro_speed;
        elseif (u3 > micro_speed_dist_thresh && u3 <= min_speed_dist_thresh)
            speed = min_speed;
        elseif (u3 > min_speed_dist_thresh && u3 <= max_speed_dist_thresh)
            speed = med_speed;
        elseif (u3 > max_speed_dist_thresh && u3 <= super_speed_dist_thresh)
            speed = max_speed;
        elseif (u3 > super_speed_dist_thresh)
            speed = super_speed;
        end
        % Rotate rover 180 CW and move forward one step
        cmdstring = [strcat('r1-',num2str(90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Reverse')
        disp(cmdstring)
       
        
        
        %Check for straight after rotation
        if u(4)-u(5)<0
            cmdstring = [strcat('r1-',num2str((atan((u(5)-u(4))/1.9551))*180/pi)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply); 
        elseif u(4)-u(5)>0 
            cmdstring = [strcat('r1-',num2str((-atan((u(5)-u(4))/1.9551))*180/pi)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        %Check for centered after rotation
        if mod(abs(u(2)-u(4)), 1)~=0
            if u(2)-u(4)>0
                cmdstring = [strcat('d1-',num2str(speed)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                disp('Move leftward')
                disp(cmdstring)
                %move left by (mod(abs(u(2)-u(4)), 1))
            elseif u(2)-u(4)<0
                cmdstring = [strcat('d1-',num2str(speed)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                disp('Move rightward')
                disp(cmdstring)
                %move left by (mod(abs(u(2)-u(4)), 1))
    end
end