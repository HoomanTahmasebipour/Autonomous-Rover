function [] = move_B(x,y)
% clear
% clc

% mat = zeros(4,8);
% mat = [ones(4,1)*NaN mat ones(4,1)*NaN];
% mat = [ones(1,10)*NaN; mat; ones(1,10)*NaN];
% 
% blocks = [3,2;2,3;3,4;3,5;1,5;1,7;3,7;5,7];
% 
% for i = 1:size(blocks,1)
%     pos = blocks(i,:);
%     mat(pos(1)+1,pos(2)+1)= NaN;
%     %mat((pos(2)*2)-1:(pos(2)*2),(pos(1)*2)-1:(pos(1)*2)) = NaN;
% end

x = x + 1;
y = y + 1;

B_mat = [7 6 5 4 NaN 0 NaN 4; 8 7 NaN 3 2 1 2 3; 9 NaN 7 NaN NaN 2 NaN 4; 8 7 6 5 4 3 NaN 5];
B_mat = [ones(4,1)*NaN B_mat ones(4,1)*NaN];
B_mat = [ones(1,10)*NaN; B_mat; ones(1,10)*NaN];

%surr = [mat(x-1,y-1) mat(x-1,y) mat(x-1,y+1); mat(x,y-1) NaN mat(x,y+1); mat(x+1,y-1) mat(x+1,y) mat(x+1,y+1)];
speed = 1;
B = 0;
surr = [B_mat(x-1,y) B_mat(x+1,y) B_mat(x,y-1) B_mat(x,y+1) ]; %f b l r
ind = find(surr == min(min(surr)));

while (B ~= 1)
    if(B_mat(x,y) == 0)
        B = 1;
        disp("In B!!!");
        disp("Please press Enter to continue");
        pause;
        break
    end
    
    if (ind == 1)
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        surr = [mat(x-1,y) mat(x+1,y) mat(x,y-1) mat(x,y+1) ]; %f b l r
        ind = find(surr == min(min(surr)));
        x = x - 1;
    elseif (ind == 2)
        cmdstring = [strcat('r1-',num2str(90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        surr = [mat(x-1,y) mat(x+1,y) mat(x,y-1) mat(x,y+1) ]; %f b l r
        ind = find(surr == min(min(surr)));
        x = x + 1;
    elseif (ind == 3)
        cmdstring = [strcat('r1-',num2str(90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        surr = [mat(x-1,y) mat(x+1,y) mat(x,y-1) mat(x,y+1) ]; %f b l r
        ind = find(surr == min(min(surr)));
        y = y - 1;
    elseif (ind == 4)
        cmdstring = [strcat('r1--',num2str(90)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        cmdstring = [strcat('d1-',num2str(speed)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        surr = [mat(x-1,y) mat(x+1,y) mat(x,y-1) mat(x,y+1) ]; %f b l r
        ind = find(surr == min(min(surr)));
        y = y + 1;
    end
end    
end