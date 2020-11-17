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

% Setup constants for algorithm
drive_straight = 0;
drive_step_counter = 0;

ultrasonic_margin = 0;
drive_margin = 0;
u1_max_dist = 2.91;
u2_max_dist = 2.48;
u3_max_dist = 2.51;
u45_max_dist = 2.57;
u6_max_dist = 3.32;
lower_thresh_unstable_drive = 0.2;
upper_thresh_unstable_drive = 5;
averaging_iters = 1;
rover_radius = 5.16;

u_in = [0,0,0,0,0,0];  % Initial ultrasonic measurements
while u_in(1) < u1_max_dist + 12
    for ct = 1:6
        cmdstring = [strcat('u',num2str(ct)) newline];
        u_in(ct) = tcpclient_write(cmdstring, s_cmd, s_rply);
    end
    
    %Rotate rover CW until driving straight
    cmdstring = [strcat('r1-',num2str(10)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
  
end
 


while 1
    
    % Take Measurements
    
    for i = 1:averaging_iters
        for ct = 1:6
            cmdstring = [strcat('u',num2str(ct)) newline];
            u(ct) = u(ct) + tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    end
    % average out the sensor values
    u = u / averaging_iters;
    
    % Display Values
    disp('Ultrasonic')
    disp(u)
    
    % u(1) is the front sensor ; u(2) is left ; u(3) is back ;
    % u(4) is right back ; u(5) is right front ; u(6) is gripper
    u4_u5_diff = abs(u(4) - u(5));
    u45 = (u(4) + u(5))/2;
    u2_u45_diff = abs(u(2) - u45);
    disp('u4_u5_diff')
    disp(u4_u5_diff)
    disp('u2_u45_diff')
    disp(u2_u45_diff)
    speed = 2;
    
    % Setup while loop flags
    forward_clear = 0;
    left_clear = 0;
    right_clear = 0;
    reverse_clear = 0;
    rover_centered = 0;
    left_too_close = 0;
    right_too_close = 0;
    unique_loc = 0;
    
    % Gather information about rover location and the available options
    if ((u(1) > 3))%u1_max_dist + u1_max_dist * ultrasonic_margin))
        forward_clear = 1;
        disp('forward_clear')
    elseif ((u(2) > 3))%u2_max_dist + u2_max_dist * ultrasonic_margin))
        left_clear = 1;
        disp('left_clear')
    elseif ((u(4) > 3) && (u(5) > 3))%u45_max_dist + u45_max_dist * ultrasonic_margin) && (u(5) > u45_max_dist + u45_max_dist * ultrasonic_margin))
        right_clear = 1;
        disp('right_clear')
    elseif ((u(3) > 3))%u3_max_dist + u3_max_dist * ultrasonic_margin))
        reverse_clear = 1;
        disp('reverse_clear')
    end
    
    
    if (u2_u45_diff < 0.3)
        rover_centered = 1;
        disp('rover_centered')
    elseif (u2_u45_diff >= 0.3)
        % Walls on either side of the rover
        rover_centered = 0;
        if (u(2) < u2_max_dist - u2_max_dist * ultrasonic_margin)
            % left side of rover too close to the maze wall
            left_too_close = 1;
            disp('left_too_close')
        elseif (u45 < u45_max_dist - u45_max_dist * ultrasonic_margin)
            % right side of rover too close to the maze wall
            right_too_close = 1;
            disp('right_too_close')
        else
            rover_centered = 1;
            disp('rover_centered')
        end
    end
   
    
    if ((u4_u5_diff < lower_thresh_unstable_drive || u4_u5_diff > upper_thresh_unstable_drive) && rover_centered == 1)
        %Drive in open direction
        if (forward_clear == 1)
            % If the way ahead is clear, build command string and move bot
            % forward
            if (u(1) <= 5)
                speed = speed / 4;
            end
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (right_clear == 1)
            % If right direction is clear, move right
            % Rotate ~ -90 degrees          
            cmdstring = [strcat('r1-',num2str(-90)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            % Drive forward 1 step
            if (u(4) <= 5 || u(5) <= 5)
                speed = speed / 4;
            end
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (left_clear == 1)
            % If left direction is clear, move left
            % Rotate ~ 90 degrees         
            cmdstring = [strcat('r1-',num2str(90)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            % Drive forward 1 step
            if (u(2) <= 5)
                speed = speed / 4;
            end
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (reverse_clear == 1)
            % If nowhere to go, reverse
            % Rotate ~ 180 degrees         
            cmdstring = [strcat('r1-',num2str(90)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            % Drive forward 1 step
            if (u(3) <= 5)
                speed = speed / 4;
            end
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (rover_centered == 0 || (u4_u5_diff >= lower_thresh_unstable_drive && u4_u5_diff <= upper_thresh_unstable_drive))
        if (rover_centered == 0 && left_too_close == 1)
            % Rotate rover -90 degrees
            cmdstring = [strcat('r1-',num2str(-90)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            % Drive to center position
            speed = speed / 10;
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            % Rotate back to starting orientation
            cmdstring = [strcat('r1-',num2str(90)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (rover_centered == 0 && right_too_close == 1)
            % Rotate rover 90 degrees
            cmdstring = [strcat('r1-',num2str(90)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            % Drive to center position
            speed = speed / 10;
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            % Rotate back to starting orientation
            cmdstring = [strcat('r1-',num2str(-90)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
        
        if (u(4) > (u(5) + u(5) * drive_margin))
            %Rotate rover CW until driving straight
            cmdstring = [strcat('r1-',num2str(10)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (u(4) < (u(5) - u(5) * drive_margin))
            %Rotate rover CCW until driving straight
            cmdstring = [strcat('r1-',num2str(-10)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    end

    stepcount = stepcount+1;
end