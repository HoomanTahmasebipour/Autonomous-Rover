function [reply] = tcpclient_write(cmd, s_cmd, s_rply)
%TCPCLIENT_WRITE writes a command string to the simulator and collects a
%response
%   Detailed explanation goes here
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

if strcmp('ua', cmd(1:2))
    %repeat 6 times for all ultrasonic sensors
    reply = zeros(1,6);
    for i = 1:6
        fwrite(s_cmd, ['u' num2str(i) newline], 'uint8')
        ct = 1;
        while ct
            if s_cmd.BytesAvailable > 0
                reply(i) = fread(s_cmd, s_cmd.BytesAvailable/4, 'single');
                ct = 0;
            end
        end
    end
else
    %original tcpclient_write
    fwrite(s_cmd, cmd, 'uint8')
    ct = 1;
    while ct
        if s_cmd.BytesAvailable > 0
            reply = fread(s_cmd, s_cmd.BytesAvailable/4, 'single');
            ct = 0;
        end
    end
end

end