
function [heading, p, k, M, ultra] = find_and_load_block(heading, s_cmd, s_rply, rover_radius, u_loc)
    [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
    deg = 10;
    tot_rot = 0;
    detect_thresh = 1.4;
    prox_thresh = 5;

    if (u(2) < u(4))
        while (u(1)/u(6) < detect_thresh) && (tot_rot < 100)
            cmdstring = [strcat('r1-',num2str(-1*deg)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            tot_rot = tot_rot + deg;
            [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
            disp(["tot_rot: " tot_rot]);
        end
    else
        while (u(1)/u(6) < detect_thresh) && (tot_rot < 100)
            cmdstring = [strcat('r1-',num2str(deg)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            tot_rot = tot_rot + deg;
            [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
            disp(["tot_rot: " tot_rot]);
        end
    end

    disp("Done Aligning");
    cmdstring = [strcat('d1-',num2str(u(6)- prox_thresh)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    disp("Move 1");
    % cmdstring = [strcat('g1-',180) newline];
    % reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    cmdstring = [strcat('d1-',num2str(prox_thresh)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    disp("Move Closer to the block by 2 inches");
    % cmdstring = [strcat('g1-',40) newline];
    % reply = tcpclient_write(cmdstring, s_cmd, s_rply);

    disp("############## Block Loaded!! ##############");
end
