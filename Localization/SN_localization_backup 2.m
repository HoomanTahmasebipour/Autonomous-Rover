
clear all
clc

%% simulation 
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
u = [0,0,0,0,0,0];
u_loc = [3.09,-1.56 ; 1.13,3.52 ; -3.49,1.04 ; -2.34375,-3.43 ; 2.34375,-3.43 ; 2.68,3.52];
pos = [0,0,0];  % Position (x,y,rotation)
rot_stuck = 90;
stepcount = 0;

rover_radius = 4;
rover_dist_thresh = 2;
ultrasonic_margin = 0.18;

% Stuck condition variables
log = 1;
stuck_cond = 0.5;
u_mat = zeros(3,6);

%% Initial Heading - 2 options
rover_centered = 0;
straighten_attempts = 0;
unique_loc = 0;
dist_tot = 0;
heading = 0;

% drive until unique location reached, and find heading (rover
% will be localized at this point)
while 1    
    % Rotate rover until sensor measurements from u4/u5 AND u1/u6 are
    % within an acceptable range
    u = take_ultrasonic_measurements(s_cmd, s_rply);
    disp('Ultrasonic Readings')
    disp(u)
    
    unique_loc = 0;
    if (u(1) > 10 && u(2) > 10 && u(3) > 10 && (u(4) > 10 || u(5) > 10))
        unique_loc = 1;
        disp('unique_loc')
    end
    
    if (unique_loc == 1)
        disp('Localized')
        if u(1) > 20 && (u(2) < 20 && u(2) > 12) && u(3) > 20  && u(4) > 20 && u(5) > 20
            heading = 0;
            disp('heading')
            disp(heading)
        elseif (u(1) < 20 && u(1) > 12) && u(2) > 20 && u(3) > 20  && u(4) > 20 && u(5) > 20
            heading = 90;
            disp('heading')
            disp(heading)
        elseif u(1) > 20 && u(2) > 20 && u(3) > 20  && (u(4) < 20 && u(4) > 12) && (u(4) < 20 && u(4) > 12)
            heading = 180;
            disp('heading')
            disp(heading)
        elseif u(1) > 20 && u(2) > 20 && (u(3) < 20 && u(3) > 12)  && u(4) > 20 && u(5) > 20
            heading = 270;
            disp('heading')
            disp(heading)
        end
    else
        if (straighten_attempts < 5)
            rover_straight = straighten_rover(u, u_loc, s_cmd, s_rply, rover_radius, unique_loc);
            straighten_attempts = straighten_attempts + 1;
        else
            straighten_attempts = 0;
            rover_straight = 1;
        end
        if (rover_straight == 1) 
            [rover_centered, unique_loc] = center_rover(u, u_loc, s_cmd, s_rply, ultrasonic_margin, rover_radius);
        end
        if ((rover_centered == 1 && rover_straight == 1) || unique_loc == 1)
            heading = move_rover(u, u_loc, s_cmd, s_rply, rover_radius, rover_dist_thresh, heading);        
        end
    end
    
    if unique_loc == 1
        break
    end
end

%% Localization - create world and initialize probability
%provide measurements and movements
k = 0;

%initalization of the world
dim1 = 32; dim2 = 16; 
locationindex = reshape(1:dim1*dim2,dim1,dim2)'; 
n = numel(locationindex);
rand('twister',5489);
bw = reshape(randi([0 1],n,1),dim2,dim1); %0 = black, 1 = white

%make blocks
M = zeros(size(bw));
Blocks = [2, 3; 3, 2; 4, 3; 5, 1; 5, 3; 7, 1; 7, 3; 7, 4;];
for xx = 1:size(Blocks,1),
	x = Blocks(xx,1); y = Blocks(xx,2);
	M(1+(y-1)*4:(y-1)*4+4, 1+(x-1)*4:(x-1)*4+4) = 1;
end
M = [ones(dim2,1) M ones(dim2,1)];
M = [ones(1, dim1+2); M; ones(1, dim1+2)];

%generate ultrasonic world
ultra = zeros(size(bw));
for sec_row = 1:4:dim2,
    for sec_col = 1:4:dim1,
        segRow = M(sec_row+2, sec_col:sec_col+5);
        segCol = M(sec_row:sec_row+5, sec_col+2);
        val = sum(segRow)+sum(segCol);
        if val == 2 && sum(segRow)~=1,
            val = 5;
        end
        ultra(sec_row:sec_row+3, sec_col:sec_col+3) = val;
    end
end

%create mask for blocks
M = abs(M-1);
M = M(2:end-1, 2:end-1);
figure; imagesc((bw+1).*M); colormap(gray);

%initialize probability
p = ones(dim2,dim1)*(1/n); 

figure;
%localizatoin loop

%% Main Rover Control Code with localization
rover_centered = 0;
straighten_attempts = 0;
unique_loc = 0;

while 1
    % Take Measurements (read from simulator or rover)
    u = take_ultrasonic_measurements(s_cmd, s_rply);
    disp('Ultrasonic Readings')
    disp(u)
    
    % update ultra
    u_dig = u(1:4) < 12;
    m_u = sum(u_dig);
    if m_u == 2 && (u_dig(1) == u_dig(3) || u_dig(2) == u_dig(4))
        m_u = 5;
    end
    disp(m_u)
    
    %sensor update
    p = sense_u(ultra, M, p, m_u);

    imagesc(p);
    title(['step: ' num2str(k)]);
    pause(0.5);
    
    unique_loc = 0;
    if (u(1) > 10 && u(2) > 10 && u(3) > 10 && (u(4) > 10 || u(5) > 10))
        unique_loc = 1;
        disp('unique_loc')
    end
    
    if (straighten_attempts < 5)
        rover_straight = straighten_rover(u, u_loc, s_cmd, s_rply, rover_radius, unique_loc);
        straighten_attempts = straighten_attempts + 1;
    else
        straighten_attempts = 0;
        rover_straight = 1;
    end
    if (rover_straight == 1) 
        [rover_centered, unique_loc] = center_rover(u, u_loc, s_cmd, s_rply, ultrasonic_margin, rover_radius);
    end
    if ((rover_centered == 1 && rover_straight == 1) || unique_loc == 1)
        heading = move_rover(u, u_loc, s_cmd, s_rply, rover_radius, rover_dist_thresh, heading);   
    end

    % movement update
    p = move(p, M, heading);
    
    disp(['heading: ' num2str(heading)])
    k = k + 1;

    % determine index location
    [loc_in_y, loc_in_x] = find(p == max(p(:)));
    disp('Possible location of rover:')
    disp(['x: ' 'y: '])
    disp([loc_in_x loc_in_y])
    
    if max(p(:)) > 0.07
        lz = 0;
        % drive to loading zone
        disp('Start driving to loading zone')
        pause(0.5);
        [lz, heading] = pathfind_to_LZ(heading, u_loc, s_cmd, s_rply, rover_radius, loc_in_x, loc_in_y, ultrasonic_margin, rover_dist_thresh);         

        if lz == 1
            disp('Loading zone reached')
         end
    end
    
end



    

    
%     if lz == 1
%         % drive to drop off zone
%         disp('Start drive to drop-off zone, press any key to continue')
%         pause;
%         while 1
%             DO = 0;
% 
%             if DO == 0
%                 DO = pathfind_to_DO(heading, u, u_loc, s_cmd, s_rply, rover_radius, loc_in_x, loc_in_y);
%             else
%                 disp('Drop off zone reached!')
%                 break
%             end
%         end
%     end

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
    
end

function rover_straight = straighten_rover(u, u_loc, s_cmd, s_rply, rover_radius, unique_loc)
    % This function ensures that after each step of the rover, it is
    % continuing to move straight
    u4 = u(4) - (rover_radius - abs(u_loc(4,2)));
    u5 = u(5) - (rover_radius - abs(u_loc(5,2)));
    disp('u4_distance, u5_distance')
    disp([u4,u5])
    lower_thresh_straight = 0.2;
    upper_thresh_straight = 5;
    u4_u5_sensor_displacement = 4.6875;
    u4_u5_diff = abs(u4 - u5);
        
    if (unique_loc == 0 && (u4_u5_diff < lower_thresh_straight || u4_u5_diff > upper_thresh_straight))
        rover_straight = 1;
        disp('Rover is straight')
    elseif (unique_loc == 1)
        rover_straight = 1;
        disp('Unique location, assuming rover is straight')
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

function heading = move_rover(u, u_loc, s_cmd, s_rply, rover_radius, rover_dist_thresh, heading)
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
        heading = mod(heading + 90,360);
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
        heading = mod(heading - 90,360);
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
        heading = heading;
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
        heading = mod(heading + 180,360);
    end
end

function [unique_loc, heading, rover_centered, rover_straight] = obstacle_avoidance(u_loc, s_cmd, s_rply, rover_radius, ultrasonic_margin, rover_dist_thresh, heading)
    rover_centered = 0;
    straighten_attempts = 0;
    unique_loc = 0;
    while 1    
        % Rotate rover until sensor measurements from u4/u5 AND u1/u6 are
        % within an acceptable range
        u = take_ultrasonic_measurements(s_cmd, s_rply);
        disp('Ultrasonic Readings')
        disp(u)

        unique_loc = 0;
        if (u(1) > 10 && u(2) > 10 && u(3) > 10 && (u(4) > 10 || u(5) > 10))
            unique_loc = 1;
            disp('unique_loc')
        end

        if (straighten_attempts < 5)
            rover_straight = straighten_rover(u, u_loc, s_cmd, s_rply, rover_radius, unique_loc);
            straighten_attempts = straighten_attempts + 1;
        else
            straighten_attempts = 0;
            rover_straight = 1;
        end
        if (rover_straight == 1) 
            [rover_centered, unique_loc] = center_rover(u, u_loc, s_cmd, s_rply, ultrasonic_margin, rover_radius);        
        end
        if ((rover_centered == 1 && rover_straight == 1) || unique_loc == 1)
            heading = move_rover(u, u_loc, s_cmd, s_rply, rover_radius, rover_dist_thresh, heading);
        end
    end
end

function pnew = move(p, mask, heading)
   %models movement error
   K = [0.1 0.1 0.1; 0.1 0.8 0.1; 0.1 0.1 0.1];
   
   %2D convolution
   pnew = conv2(p, K,'same');
   
   col_move = cosd(heading);
   row_move = sind(heading);
   
   if col_move > 0
	   pnew = [zeros(size(pnew,1),1) pnew(:,1:end-1)];
   elseif col_move < 0
	   pnew = [pnew(:,2:end) zeros(size(pnew,1),1)];
   end
   
   if row_move < 0
	   pnew = [zeros(1,size(pnew,2)); pnew(1:end-1,:)];
   elseif row_move > 0
	   pnew = [pnew(2:end,:); zeros(1,size(pnew,2))];
   end

   %masking matrix
   pnew = pnew.*mask;

   %normalization
   pnew = pnew/sum(sum(pnew));
  
end

function pnew = sense_u(world, mask, p, SenVal)
  %models sensor reading
  pHit = 0.6; %default = 0.9
  pMiss = 0.2; %default = 0.1
  
  mult = pMiss*ones(size(world));
  mult(world == SenVal) = pHit;
  
  %2D multiplication step
  pnew = p.*mult;
  
  %masking matrix
  pnew = pnew.*mask;
   
  %normalization
  pnew = pnew./sum(sum(pnew));

end

function [lz, heading] = pathfind_to_LZ(heading, u_loc, s_cmd, s_rply, rover_radius, loc_in_x, loc_in_y, ultrasonic_margin, rover_dist_thresh)
    % This function gives directions to the rover to find the loading zone
    nan(1:4,1:4) = NaN;
    z = zeros(4,4);
    lz = 0;
    
    lz_dist = [z z z+1 z+2 nan z+6 nan z+8; z z nan z+3 z+4 z+5 z+6 z+7; z+1 nan z+5 nan nan z+6 nan z+8; z+2 z+3 z+4 z+5 z+6 z+7 nan z+9];
    dist_to_lz = lz_dist(loc_in_y, loc_in_x);
    disp(['distance to loading zone: ' num2str(dist_to_lz)])
    
   
end

function DO = pathfind_to_DO(heading, u, u_loc, s_cmd, s_rply, rover_radius, loc_in_x, loc_in_y)
    imshow('dropoff_zones.png')
    drop_off = input('Enter dropoff location (refer to image): ', 's');
    nan(1:4,1:4) = NaN;
    z = zeros(4,4);
    A = [z+6 z+7 z+8 z+9 nan z+7 nan z+9; z+5 z+6 nan z+8 z+7 z+6 z+7 z+8; z+4 nan z nan nan z+5 nan z+9; z+3 z+2 z+1 z+2 z+3 z+4 nan z+10];
    B = [z+7 z+6 z+5 z+4 nan z nan z+4; z+8 z+7 nan z+3 z+2 z+1 z+2 z+3; z+9 nan z+7 nan nan z+2 nan z+4; z+8 z+7 z+6 z+5 z+4 z+3 nan z+5];
    C = [z+9 z+8 z+7 z+6 nan z+4 nan z; z+10 z+9 nan z+5 z+4 z+3 z+2 z+1; z+11 nan z+9 nan nan z+4 nan z+2; z+10 z+9 z+8 z+7 z+6 z+5 nan z+3];
    D = [z+10 z+9 z+8 z+7 nan z+5 nan z+3; z+11 z+10 nan z+6 z+5 z+4 z+3 z+2; z+12 nan z+10 nan nan z+5 nan z+1; z+11 z+10 z+9 z+8 z+7 z+6 nan z];
    
    DO = 0;
    
end