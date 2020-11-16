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

ultrasonic_margin_err = 0.05;
drive_margin_err = 0;
u1_max_dist = 2.91;
u2_max_dist = 2.48;
u3_max_dist = 2.51;
u4_max_dist = 2.57;
u5_max_dist = 2.57;
u6_max_dist = 3.32;
lower_thresh_unstable_drive = 0.1;
upper_thresh_unstable_drive = 5;
averaging_iters = 4;
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
    disp('u4_u5_diff')
    disp(u4_u5_diff)
    speed = 2;
    forward_clear = 0;
    left_clear = 0;
    right_clear = 0;
    reverse_clear = 0;
    
    if ((u(1) > u1_max_dist + u1_max_dist * ultrasonic_margin_err))
        forward_clear = 1;
    elseif ((u(2) > u2_max_dist + u2_max_dist * ultrasonic_margin_err))
        left_clear = 1;
    elseif ((u(4) > u4_max_dist + u4_max_dist * ultrasonic_margin_err) && (u(5) > u5_max_dist + u5_max_dist * ultrasonic_margin_err))
        right_clear = 1;
    elseif ((u(3) > u3_max_dist + u3_max_dist * ultrasonic_margin_err))
        reverse_clear = 1;
    end
    
    if (u4_u5_diff < lower_thresh_unstable_drive || u4_u5_diff > upper_thresh_unstable_drive)
        %Drive in open direction
        if (forward_clear == 1)
            % If the way ahead is clear, build command string and move bot
            % forward
            if (u(1) <= 5)
                speed = speed / 4;
            end
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (left_clear == 1)
            % If left direction is clear, move left
            % Rotate CW ~ 90 degrees         
            cmdstring = [strcat('r1-',num2str(90)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            % Drive forward 1 step
            if (u(2) <= 5)
                speed = speed / 4;
            end
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (right_clear == 1)
            % If right direction is clear, move right
            % Rotate CW ~ -90 degrees          
            cmdstring = [strcat('r1-',num2str(-90)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            % Drive forward 1 step
            if (u(4) <= 5 || u(5) <= 5)
                speed = speed / 4;
            end
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (reverse_clear == 1)
            % If nowhere to go, reverse
            % Rotate CW ~ 180 degrees         
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
    elseif (u4_u5_diff >= lower_thresh_unstable_drive && u4_u5_diff <= upper_thresh_unstable_drive)
        if (u(4) > (u(5) + u(5) * drive_margin_err))
            %Rotate rover CW until driving straight
            cmdstring = [strcat('r1-',num2str(-1)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (u(4) < (u(5) - u(5) * drive_margin_err))
            %Rotate rover CCW until driving straight
            cmdstring = [strcat('r1-',num2str(1)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    end
    
    stepcount = stepcount+1;
end