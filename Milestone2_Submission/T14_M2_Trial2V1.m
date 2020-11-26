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

% Global Variables
u = [0,0,0,0,0,0];
u_loc = [3.09,-1.56 ; 1.13,3.52 ; -3.49,1.04 ; -2.34375,-3.43 ; 2.34375,-3.43 ; 2.68,3.52];
pos = [0,0,0];  % Position (x,y,rotation)
rot_stuck = 90;
stepcount = 0;

rover_radius = 4;
rover_dist_thresh = 2;
ultrasonic_margin = 0.18;
disp("The rover is attempting to track its location in this version, to ensure localization is working as expected")
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

%% Straighten rover and determine it's initial heading
rover_centered = 0;
straighten_attempts = 0;
unique_loc = 0;
dist_tot = 0;
rover_straight = 0;

% straighten rover and take compass readings to determine heading
while rover_straight == 0
    % Rotate rover until sensor measurements from u4/u5 are
    % within an acceptable range
    [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
    
    if (straighten_attempts < 5)
        rover_straight = straighten_rover(u, s_cmd, s_rply, unique_loc);
        straighten_attempts = straighten_attempts + 1;
    else
        straighten_attempts = 0;
        rover_straight = 1;
    end
end

comp = tcpclient_write(['c1' newline], s_cmd, s_rply);
disp(comp)
if (sim == 1)
    if comp < 45 && comp > -45
        heading = 0;
    elseif comp < 135 && comp > 45
        heading = 90;
    elseif comp < -135 || comp > 135
        heading = 180;
    elseif comp > -135 && comp < -45
        heading = 270;
    end
else
    if comp < 45 || comp > 315
        heading = 0;
    elseif comp < 135 && comp > 45
        heading = 90;
    elseif comp < 225 && comp > 135
        heading = 180;
    elseif comp < 315 && comp > 225
        heading = 270;
    end 
end
disp(['Heading: ' num2str(heading)])

%% Main Rover Control Code with localization
rover_centered = 0;
straighten_attempts = 0;
unique_loc = 0;
localized = 0;
old_heading = heading;
while (localized == 0)    
    % Take Measurements (read from simulator or rover)
    disp(['------------------------- Step Count: ' num2str(k) ' -------------------------'])
    [u, u_real] = take_ultrasonic_measurements(s_cmd, s_rply, rover_radius, u_loc);
    disp('Ultrasonic Measurements')
    disp(u)
    
    if (straighten_attempts < 5 && old_heading ~= heading)
        rover_straight = straighten_rover(u, s_cmd, s_rply, unique_loc);
        straighten_attempts = straighten_attempts + 1;
    else
        straighten_attempts = 0;
        rover_straight = 1;
    end
        
    if (rover_straight == 1)
        p = update_localization_map(u, M, p, ultra, k);
    end
    
    unique_loc = 0;
    if (u_real(1) > 10 && u_real(2) > 10 && u_real(3) > 10 && (u_real(4) > 10 || u_real(5) > 10))
        unique_loc = 1;
        disp('unique_loc')
    end
    
    if (rover_straight == 1)
        rover_centered = center_rover(u, s_cmd, s_rply, ultrasonic_margin, rover_radius, unique_loc);
    end
    if ((rover_centered == 1 && rover_straight == 1) || unique_loc == 1)
        old_heading = heading;
        heading = move_rover(u, s_cmd, s_rply, rover_dist_thresh, heading);
    end
    
    if (rover_straight == 1)
        [p,k,loc_y,loc_x, localized] = update_rover_location(p, M, heading, k);
    end
end
%% Helper functions
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

function rover_straight = straighten_rover(u, s_cmd, s_rply, unique_loc)
    % This function ensures that after each step of the rover, it is
    % continuing to move straight
    lower_thresh_straight = 0.2;
    upper_thresh_straight = 5;
    u4_u5_sensor_displacement = 4.6875;
    u4_u5_diff = abs(u(4) - u(5));
        
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
        if (u(4) > u(5))
            disp(['Straighten rover: turn in postive direction by: ' num2str(rot)])
            cmd = strcat('r1-',num2str(rot)); 
            cmdstring = [cmd newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (u(4) <= u(5))
            disp(['Straighten rover: turn in negative direction by: ' num2str(rot)])
            cmd = strcat('r1--',num2str(rot));
            cmdstring = [cmd newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    end 
end

function rover_centered = center_rover(u, s_cmd, s_rply, ultrasonic_margin, rover_radius, unique_loc)
    % This function centers the rover as much as possible to ensure
    % straight motion
    u_left = u(2);
    if (u(4) < u(5))
        u_right = u(4);
    else
        u_right = u(5);
    end
    centered_dist_from_wall = 6 - rover_radius;
    centered_dist_from_wall_min = centered_dist_from_wall - centered_dist_from_wall * ultrasonic_margin;
    centered_dist_from_wall_max = centered_dist_from_wall + centered_dist_from_wall * ultrasonic_margin;
        
    if (unique_loc == 0 && ((u_left > centered_dist_from_wall_min && u_left < centered_dist_from_wall_max) || (u_right > centered_dist_from_wall_min && u_right < centered_dist_from_wall_max)))
        rover_centered = 1;
        disp('rover_centered')
    elseif (unique_loc == 0 && (u_left < centered_dist_from_wall_min || (u_right < u_left && u_right > centered_dist_from_wall_max)))
        % Either left side is too close or right side is too far, move right
        angle = 22;
        adjustment = u_right / 2;
        req_dist = adjustment / tand(angle);
        speed = adjustment / sind(angle);
        if (speed > 3)
            speed = 3;
        end
        if (u(1) > req_dist)
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
            disp('Left/Right side too close/far but cant move to fix. Moving back half an inch and assuming centered')
            cmdstring = [strcat('a1-',num2str(0.5)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (unique_loc == 0 && (u_right < centered_dist_from_wall_min || (u_left < u_right && u_left > centered_dist_from_wall_max)))
        % Eithert right side is too close or left side is too far, move left
        angle = 22;
        adjustment = u_left / 2;
        req_dist = adjustment / tand(angle);
        speed = adjustment / sind(angle);
        if (speed > 3)
            speed = 3;
        end
        if (u(1) > req_dist) 
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
            disp('Right/Left side too close/far but cant move to fix. Moving back half an inch and assuming centered')
            cmdstring = [strcat('a1-',num2str(0.5)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    elseif (unique_loc == 1)
        rover_centered = 1;
        disp('Unique location, assuming rover_centered')
    else
        rover_centered = 1;
    end
end

function heading = move_rover(u, s_cmd, s_rply, rover_dist_thresh, heading)
    % This function's purpose is to move the rover one step in the optimal
    % direction
    u45 = abs(u(4) + u(5))/2;
        
    if (u(2) > u45 && u(2) > rover_dist_thresh*1.5 && u(1) <= rover_dist_thresh*1.25)
        % Move left, first determine rover speed
        speed = u(2) / 2;
        if (speed > 6)
            speed = 6;
        end
        
        % Rotate rover 90 CW and move forward one step
        cmdstring = [strcat('r1-',num2str(90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp(['Move left ' num2str(speed) ' inches'])
        heading = mod(heading + 90,360);
    elseif (u45 >= u(2) && u45 >= rover_dist_thresh*1.5 && u(1) <= rover_dist_thresh*1.25)
        % Move right, first determine rover speed
        if (u(4) <= u(5))
            speed = u(4) / 2;
        elseif (u(5) < u(4))
            speed = u(5) / 2;
        end
        if (speed > 6)
            speed = 6;
        end
        
        % Rotate rover 90 CCW and move forward one step
        cmdstring = [strcat('r1-',num2str(-90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp(['Move right ' num2str(speed) ' inches'])
        heading = mod(heading - 90,360);
    elseif (u(1) > rover_dist_thresh*1.25)
        % Move forward, first determine rover speed
        speed = u(1) / 2;
        if (speed > 6)
            speed = 6;
        end
        
        % Move rover forward one step
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp(['Move forward ' num2str(speed) ' inches'])
        heading = heading;
    elseif (u(3) > rover_dist_thresh)
        % Move backwards, first determine rover speed
        speed = u(3) /2;
        if (speed > 6)
            speed = 6;
        end
        
        % Rotate rover 180 CW and move forward one step
        cmdstring = [strcat('r1-',num2str(180)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        disp(['Reverse ' num2str(speed) ' inches'])
        heading = mod(heading + 180,360);
    end
end

function p = update_localization_map(u, M, p, ultra, k)
    % Determine rover location
    u_dig = u(1:4) < 12;
    disp('u_dig')
    disp(u_dig)
    m_u = sum(u_dig);
    if m_u == 2 && (u_dig(1) == u_dig(3) || u_dig(2) == u_dig(4))
        m_u = 5;
    end
    
    %sensor update
    p = sense_u(ultra, M, p, m_u);
    
    imagesc(p);
    colorbar
    title(['step: ' num2str(k)]);
    pause(0.5);
end

function [p,k,loc_y,loc_x, localized] = update_rover_location(p, M, heading, k)
    % movement update
    p = move(p, M, heading);
    
    disp(['heading: ' num2str(heading)])
    k = k + 1;

    % determine index location
    %[loc_y, loc_x] = find(p == max(p(:)));
    [val, index] = max(p(:));
    [loc_y,loc_x] = ind2sub(size(p),index);
    disp('Possible location of rover (x, y):')
    disp([loc_x loc_y p(loc_y, loc_x)])
    if (p(loc_y, loc_x) > 0.07)
        localized = 0;
    else
        localized = 0;
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