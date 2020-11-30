clear
close all
clc

%% Rover/Simulation Initialization and Setup
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

%% Find and pickup Block
find_and_load_block(s_cmd, s_rply, rover_radius, u_loc, rover_dist_thresh)

%% Drop block
deliver_block_and_close_gate(s_cmd, s_rply)

function find_and_load_block(s_cmd, s_rply, rover_radius, u_loc, rover_dist_thresh)
    [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
    deg = 10;
    tot_rot = 0;
    detect_thresh = 1.4;
    prox_thresh = 6.5;
    if (u(2) < u(4))
        while (u(1)/u(6) < detect_thresh) && (abs(tot_rot) < 100)
            rot = -1*deg;
            cmdstring = [strcat('r1-',num2str(rot)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            tot_rot = tot_rot + rot;
            [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
            disp(["tot_rot: " tot_rot]);
        end
    else
        while (u(1)/u(6) < detect_thresh) && (abs(tot_rot) < 100)
            rot = deg;
            cmdstring = [strcat('r1-',num2str(rot)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            tot_rot = tot_rot + rot;
            [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
            disp(["tot_rot: " tot_rot]);
        end
    end

    disp("Found block!");
    speed = u(6) - prox_thresh;
    disp(['Giving gripper enough space to open, moving: ' speed])
    if (speed < 0)
        cmdstring = [strcat('a1-',num2str(abs(speed))) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    else
        cmdstring = [strcat('d1-',num2str(abs(speed))) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    end
    [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
    
    cmdstring = [strcat('g1-',num2str(180)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    
    speed = 3;
    disp(['Move towards the block, and pick it up. Move: ' speed])
    cmdstring = [strcat('d1-',num2str(prox_thresh)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
    cmdstring = [strcat('g1-',num2str(40)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    
    cmdstring = [strcat('r1-',num2str(-1*tot_rot)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
    
    while (u(1) > rover_dist_thresh*1.25)
        speed = u(1) / 2;
        if (speed > 3)
            speed = 3;
        end
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
    end
    
    disp("############## Block Loaded!! ##############");
end

function deliver_block_and_close_gate(s_cmd, s_rply)
    disp("Unloading block!");
    cmdstring = [strcat('g1-',num2str(180)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    
    speed = 6;
    disp('Moving rover back 6 inches to close gate')
    cmdstring = [strcat('a1-',num2str(speed)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    disp('Close gripper gate')
    cmdstring = [strcat('g1-',num2str(40)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
end

function [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc)
    % This function takes ultrasonic readings as many as num_measurements,
    % averages them out, and returns them in a 1x6 vector.
    u = [0,0,0,0,0,0];
    u_real = [0,0,0,0,0,0];
    cmdstring = ['ua' newline];
    u_real = tcpclient_write(cmdstring, s_cmd, s_rply);
    u1 = u_real(1) - (rover_radius - abs(u_loc(1,1)));
    u2 = u_real(2) - (rover_radius - abs(u_loc(2,2)));
    u3 = u_real(3) - (rover_radius - abs(u_loc(3,1)));
    u4 = u_real(4) - (rover_radius - abs(u_loc(4,2)));
    u5 = u_real(5) - (rover_radius - abs(u_loc(5,2)));
    u6 = u_real(6) - (rover_radius - abs(u_loc(6,1)));
    u = [u1 u2 u3 u4 u5 u6];
end