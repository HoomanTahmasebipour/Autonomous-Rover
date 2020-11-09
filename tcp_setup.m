function [s_cmd, s_rply] = tcp_setup(role,port1,port2)
%TCP_SETUP Sets up a TCP connection for the simulator
%   Defaults to client role, ports 9000 and 9001
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

if ~exist('role','var')
    role = 'client';
end
if ~exist('port1','var')
    port1 = 9000;
end
if ~exist('port2','var')
    port2 = 9001;
end

% Open and begin listening to TCP connections for data
s_cmd = tcpip('localhost',port1, 'NetworkRole', role);

s_rply = tcpip('localhost',port2, 'NetworkRole', role);

end

