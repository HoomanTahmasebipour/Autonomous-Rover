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
% u1 - front sensor reading
% u2 - right back sensor reading
% u3 - right front sensor reading
% u4 - back sensor reading
% u5 - left sensor reading
% u6 - front lower sensor reading
pos = [0,0,0];  % Position (x,y,rotation)
speed = 2;
rot_stuck = 90;
stepcount = 0;
d = 1.973; % distance between u2 and u3 (inches)



while 1
    
    %% Take Measurements
    for ct = 1:6
        cmdstring = [strcat('u',num2str(ct)) newline];
        u(ct) = tcpclient_write(cmdstring, s_cmd, s_rply);
    end
    
%     ir = tcpclient_write(['i1' newline], s_cmd, s_rply);
    
    odom = tcpclient_write(['o3' newline], s_cmd, s_rply);
    
%     gyro = tcpclient_write(['g1' newline], s_cmd, s_rply);
%     
%     comp = tcpclient_write(['c1' newline], s_cmd, s_rply);
    
    % Display Values
    disp('Ultrasonic [front, right (front), right (left), back, left')
    disp(u)
%     disp('IR Sensor')
%     disp(ir)
    disp('Odometer')
    disp(odom)
%     disp('Gyroscope')
%     disp(gyro)
%     disp('Compass')
%     disp(comp)
    
    %% if rover is driving too close to a wall, adjust if too close to one side
    % rover in between 2 walls
    for ct = 1:2
        if (u(5) < 4) && (u(2) < 4)
            if (u(5) - u(3)) > 1.5  % close to right wall
                % turn -45deg
                cmdstring = [strcat('r1-',num2str(45)) newline];  % Rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);

                % drive 1 in forward
                cmdstring = [strcat('d1-',num2str(1)) newline];             
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);

                % turn 45deg to reposition
                cmdstring = [strcat('r1-',num2str(-35)) newline];  % Rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);

                break

            elseif (u(3) - u(5)) > 1.5  % close to left wall
                % turn 45deg
                cmdstring = [strcat('r1-',num2str(-45)) newline];  % Rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);

                % drive 1 in forward
                cmdstring = [strcat('d1-',num2str(1)) newline];             
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);

                % turn -45deg to reposition
                cmdstring = [strcat('r1-',num2str(35)) newline];  % Rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);

                break
            else
                % do nothing
                break

            end
        elseif (u(5) < 1.64) && (u(2) > 2) % close to left wall (but no wall on right)
            % turn 45deg
            cmdstring = [strcat('r1-',num2str(-45)) newline];  % Rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);

            % drive 1 in forward
            cmdstring = [strcat('d1-',num2str(1)) newline];             
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);

            % turn -45deg to reposition
            cmdstring = [strcat('r1-',num2str(35)) newline];  % Rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);

            break
        elseif (u(5) > 2) && (u(2) < 1.73) % close to right wall (but no wall on left)
            % turn -45deg
            cmdstring = [strcat('r1-',num2str(45)) newline];  % Rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);

            % drive 1 in forward
            cmdstring = [strcat('d1-',num2str(1)) newline];             
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);

            % turn 45deg to reposition
            cmdstring = [strcat('r1-',num2str(-35)) newline];  % Rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);

            break
        end
    end
    
    %% Check if driving straight
    % check right side sensor values
    for ct = 1:2
        if ((u(3) - .1) < u(2)) && (u(2) < (u(3) + .1)) % values are same within a range
            % do nothing
            break
        elseif (u(3) > u(2)) && (u(3) - u(2) < d) % front is away from wall
            % turn right (positive angle)
            theta = asin((u(3) - u(2)) / d);

            cmdstring = [strcat('r1-',num2str(theta)) newline];  % Rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);

            break

        elseif u(3) < u(2)  && (u(2) - u(3) < d) % front is towards wall
            % turn left (negative angle)
            theta = asin((u(3) - u(2)) / d);

            cmdstring = [strcat('r1-',num2str(theta)) newline];  % Rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);

            break
        end
    end

        
    %% drive
    for ct = 1:2
        if  (u(1) > 2.91) && (u(2) > 1) && (u(5) > 1) % front clear

            % Drive forward
            cmdstring = [strcat('d1-',num2str(speed)) newline];             
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);

            break

        elseif u(1) < 2.91 % obstacle in front

            if (u(2) < 6) && (u(5) < 6) % obstacle on right and left, rotate 180

                cmdstring = [strcat('r1-',num2str(180)) newline];  % Rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);

                % Drive forward
                cmdstring = [strcat('d1-',num2str(speed)) newline];             
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                break

            elseif (u(5) > 4) && (u(3) > 6) % no obstacles on sides

                if u(5) > u(3)  % left sensor value greater than right sensor value, rotate left

                    cmdstring = [strcat('r1-',num2str(90)) newline];  % Rotate bot
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);

                    % Drive forward
                    cmdstring = [strcat('d1-',num2str(speed)) newline];             
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                    break

                elseif u(5) < u(3)  % right sensor value greater than left sensor value, rotate right

                    cmdstring = [strcat('r1-',num2str(-90)) newline];  % Rotate bot
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);

                    % Drive forward
                    cmdstring = [strcat('d1-',num2str(speed)) newline];             
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                    break

                end

                break

            elseif u(5) < 6 % obstacle on left, rotate right

                cmdstring = [strcat('r1-',num2str(-90)) newline];  % Rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);

                % Drive forward
                cmdstring = [strcat('d1-',num2str(speed)) newline];             
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                break

            elseif u(3) < 6 % obstacle on right, rotate left

                cmdstring = [strcat('r1-',num2str(90)) newline];  % Rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);

                % Drive forward
                cmdstring = [strcat('d1-',num2str(speed)) newline];             
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                break

            end
            break         
        end
    end
    stepcount = stepcount+1;
    
end