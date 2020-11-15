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
u2_max_dist = 2.57;
u4_max_dist = 2.51;
u5_max_dist = 2.48;
while 1
    
    % Take Measurements
    for ct = 1:6
        cmdstring = [strcat('u',num2str(ct)) newline];
        u(ct) = tcpclient_write(cmdstring, s_cmd, s_rply);
    end
    
    % Display Values
    disp('Ultrasonic')
    disp(u)
    
    % u(1) is the front sensor ; u(2) is front right ; u(3) is back right ;
    % u(4) is back ; u(5) is left ; u(6) is left
    % TODO: CURRENT PROBLEM - drive control not working well. change in
    % ultrasonic sensor value when passing an edge isnt being reflected accuratley
    u2_u3_diff = abs(u(2) - u(3))
    speed = 2;
    
    if (drive_straight == 1)
        %Drive in open direction
        if ((u(1) > u1_max_dist + u1_max_dist * ultrasonic_margin_err))
            % If the way ahead is clear, build command string and move bot
            % forward
            if (u(1) <= 5)
                speed = speed / 4;
            end
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif ((u(2) > u2_max_dist + u2_max_dist * ultrasonic_margin_err))
            % Rotate CW ~ 90 degrees         
            cmdstring = [strcat('r1-',num2str(-90)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            % Drive forward 1 step
            if (u(2) <= 5)
                speed = speed / 4;
            end
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif ((u(5) > u5_max_dist + u5_max_dist * ultrasonic_margin_err))
             % Rotate CW ~ -90 degrees          
            cmdstring = [strcat('r1-',num2str(90)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            % Drive forward 1 step
            if (u(5) <= 5)
                speed = speed / 4;
            end
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif ((u(4) > u4_max_dist + u4_max_dist * ultrasonic_margin_err))
            % Rotate CW ~ 180 degrees         
            cmdstring = [strcat('r1-',num2str(90)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            % Drive forward 1 step
            if (u(4) <= 5)
                speed = speed / 4;
            end
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end

        
            
    elseif (drive_straight == 0 && (u(2) <= u(3) + u(3) * drive_margin_err) && (u(2) >= u(3) - u(3) * drive_margin_err) || u2_u3_diff > 5 || u2_u3_diff < 0.1)
        drive_straight = 1;
    elseif (drive_straight == 0 && ((u(2) > u(3) + u(3) * drive_margin_err) || (u(2) < u(3) - u(3) * drive_margin_err)) && u2_u3_diff <= 5)
        if (u(2) > (u(3) + u(3) * drive_margin_err))
            %Rotate rover CW until driving straight
            cmdstring = [strcat('r1-',num2str(-1)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (u(2) < (u(3) - u(3) * drive_margin_err))
            %Rotate rover CCW until driving straight
            cmdstring = [strcat('r1-',num2str(1)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    end
    
    stepcount = stepcount+1;
end