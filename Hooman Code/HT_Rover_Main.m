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
rover_dist_thresh = 1;
ultrasonic_margin = 0.18;

% Stuck condition variables
log = 1;
stuck_cond = 0.5;
u_mat = zeros(3,6);

%% Main Rover Control Code
rover_centered = 0;

while 1    
    % Rotate rover until sensor measurements from u4/u5 AND u1/u6 are
    % within an acceptable range
    u = take_ultrasonic_measurements(s_cmd, s_rply);
    disp('Ultrasonic Readings')
    disp(u)
    unique_loc = 0;
    rover_straight = straighten_rover(u, u_loc, s_cmd, s_rply, rover_radius);
    if (rover_straight == 1) 
        [rover_centered, unique_loc] = center_rover(u, u_loc, s_cmd, s_rply, ultrasonic_margin, rover_radius);
    end
    if ((rover_centered == 1 && rover_straight == 1) || unique_loc == 1)
        move_rover(u, u_loc, s_cmd, s_rply, rover_radius, rover_dist_thresh)
    end
end
%% Helper functions
function [u] = take_ultrasonic_measurements(s_cmd, s_rply)
    % This function takes ultrasonic readings as many as num_measurements,
    % averages them out, and returns them in a 1x6 vector.
    u = [0,0,0,0,0,0];
    num_measurements = 1;
    
    for i = 1:num_measurements
        cmdstring = ['ua' newline];
        u = tcpclient_write(cmdstring, s_cmd, s_rply);
    end
    
    % average out the sensor values
    u = u / num_measurements;
end

function rover_straight = straighten_rover(u, u_loc, s_cmd, s_rply, rover_radius)
    % This function ensures that after each step of the rover, it is
    % continuing to move straight
    u4 = u(4) - (rover_radius - abs(u_loc(4,2)));
    u5 = u(5) - (rover_radius - abs(u_loc(5,2)));
    disp('u4_distance, u5_distance')
    disp([u4,u5])
    lower_thresh_straight = 0.2;
    upper_thresh_straight = 1;
    u4_u5_sensor_displacement = 2.23;
    u4_u5_diff = abs(u4 - u5);
        
    if ((u4_u5_diff < lower_thresh_straight || u4_u5_diff > upper_thresh_straight))
        rover_straight = 1;
        disp('Rover is straight')
    else
        % Rover is not straight
        disp('Rover is not straight')
        rover_straight = 0;
        % Determine number of degrees to turn rover
        rot = atand(u4_u5_diff / u4_u5_sensor_displacement) / 2;
        
        % Determine if to turn rover CW or CCW
        if (u4 > u5)
            disp('Straighten rover: turn in postive direction by:')
            disp(rot)
            cmd = strcat('r1-',num2str(rot)); 
            cmdstring = [cmd newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (u4 <= u5)
            disp('Straighten rover: turn in negative direction by:')
            disp(rot)
            cmd = strcat('r1--',num2str(rot));
            cmdstring = [cmd newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    end 
end

function [rover_centered, unique_loc] = center_rover(u, u_loc, s_cmd, s_rply, ultrasonic_margin, rover_radius)
    % This function centers the rover as much as possible to ensure
    % straight motion
    u1 = u(1) - (rover_radius - abs(u_loc(1,1)));
    u2 = u(2) - (rover_radius - abs(u_loc(2,2)));
    u3 = u(3) - (rover_radius - abs(u_loc(3,1)));
    u4 = u(4) - (rover_radius - abs(u_loc(4,2)));
    u5 = u(5) - (rover_radius - abs(u_loc(5,2)));
    u_left = u2;
    if (u4 < u5)
        u_right = u4;
    else
        u_right = u5;
    end
    disp('u_left_distance,u_right_distance')
    disp([u_left,u_right])
    centered_dist_from_wall = 6 - rover_radius;
    centered_dist_from_wall_min = centered_dist_from_wall - centered_dist_from_wall * ultrasonic_margin;
    centered_dist_from_wall_max = centered_dist_from_wall + centered_dist_from_wall * ultrasonic_margin;
    unique_loc = 0;
    if (u_left > 10 && u_right > 10 && u1 > 10 && u3 > 10)
        unique_loc = 1;
        disp('unique_loc')
    end
        
    if (unique_loc == 0 && ((u_left > centered_dist_from_wall_min && u_left < centered_dist_from_wall_max) || (u_right > centered_dist_from_wall_min && u_right < centered_dist_from_wall_max)))
        rover_centered = 1;
        disp('rover_centered')
    elseif (unique_loc == 0 && (u_left < centered_dist_from_wall_min || (u_right < u_left && u_right > centered_dist_from_wall_max)))
        % Either left side is too close or right side is too far, move right
        angle = 22;
        adjustment = u_right / 2;
        req_dist = adjustment / tand(angle);
        speed = adjustment / sind(angle);
        
        if (u1 > req_dist)
            cmdstring = [strcat('r1-',num2str(-angle)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('r1-',num2str(angle)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            disp('Left/Right side too close/far, move right. r1-22, d1-1, r1--22')
            rover_centered = 0;
        else
            rover_centered = 1;
            disp('Left/Right side too close/far but cant move to fix. Assuming centered')
        end
    elseif (unique_loc == 0 && (u_right < centered_dist_from_wall_min || (u_left < u_right && u_left > centered_dist_from_wall_max)))
        % Eithert right side is too close or left side is too far, move left
        angle = 22;
        adjustment = u_left / 2;
        req_dist = adjustment / tand(angle);
        speed = adjustment / sind(angle);
        
        if (u1 > req_dist) 
            cmdstring = [strcat('r1-',num2str(angle)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('r1-',num2str(-angle)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            disp('Right/Left side too close/far, move left. r1--22, d1-1, r1-22')
            rover_centered = 0;
        else
            rover_centered = 1;
            disp('Right/Left side too close/far but cant move to fix. Assuming centered')
        end
    elseif (unique_loc == 1)
        rover_centered = 1;
        disp('Unique location, assuming rover_centered')
    end
end

function move_rover(u, u_loc, s_cmd, s_rply, rover_radius, rover_dist_thresh)
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
    u45 = abs(u4 + u5)/2;
        
    if (u2 > u45 && u2 > rover_dist_thresh*1.75 && u1 <= rover_dist_thresh)
        % Move left, first determine rover speed
        speed = u2 / 2;
        if (speed > 6)
            speed = 6;
        end
        
        % Rotate rover 90 CW and move forward one step
        cmdstring = [strcat('r1-',num2str(90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Move left')
        disp(cmdstring)
    elseif (u45 >= u2 && u45 >= rover_dist_thresh*1.75 && u1 <= rover_dist_thresh)
        % Move right, first determine rover speed
        if (u4 <= u5)
            speed = u4 / 2;
        elseif (u5 < u4)
            speed = u5 / 2;
        end
        if (speed > 6)
            speed = 6;
        end
        
        % Rotate rover 90 CCW and move forward one step
        cmdstring = [strcat('r1-',num2str(-90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Move right')
        disp(cmdstring)
    elseif (u1 > rover_dist_thresh*1.25)
        % Move forward, first determine rover speed
        speed = u1 / 2;
        if (speed > 6)
            speed = 6;
        end
        
        % Move rover forward one step
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp('Move forward')
        disp(cmdstring)
    elseif (u3 > rover_dist_thresh)
        % Move backwards, first determine rover speed
        speed = u3 /2;
        if (speed > 6)
            speed = 6;
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

function check_stuck_condition(u, u_loc, s_cmd, s_rply, log, u_mat, stuck_cond, rover_radius, rover_dist_thresh)
    u3 = u(3) - (rover_radius - abs(u_loc(3,1)));
    if (log < 3)
        u_mat(log,:) = u;
        log = log + 1;
    elseif (log == 3)
        u_mat(log,:) = u;
        log = 1;
        umat1_umat2_diff = abs(max(u_mat(1,:) - u_mat(2,:)));
        umat1_umat3_diff = abs(max(u_mat(1,:) - u_mat(3,:)));
        umat2_umat3_diff = abs(max(u_mat(2,:) - u_mat(3,:)));
        if (umat1_umat2_diff < stuck_cond && umat1_umat3_diff < stuck_cond && umat2_umat3_diff < stuck_cond )
            % Check if theres space to move back
            if (u3 > rover_dist_thresh)
                disp("Stuck, but finding way out");
                speed = u3 / 2;
                if (speed > 10)
                    speed = 10;
                end
                % Move backwards and reorient, try again
                cmdstring = [strcat('a1-',num2str(speed)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);         
                cmdstring = [strcat('r1-',num2str(-10)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            else
                disp("Stuck and I need help")
            end
        end
    end
end