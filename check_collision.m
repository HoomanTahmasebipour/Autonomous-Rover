function [collision] = check_collision(bot_pos, maze)
%CHECK_COLLISION checks for collision between robot and maze walls
%   This function takes in the coordinate definitions of the maze walls and
%   the robot perimeter, and checks whether there are any collisions
%   between them. It checks whether the robot perimeter points are within
%   the walls, and whether any wall perimeter points are inside the robot,
%   for robustness.
%   
%   Input
%   bot_pos - the perimeter points of the robot body, global coordinates (X,Y)
%   maze - perimeter points of the maze, global coordinates (X,Y). The maze
%   outer perimeter must be defined counterclockwise and the interior walls
%   defined clockwise. Only works with polygonal wall segments.
%   
%   Output
%   collision - logical "1" if there is a collision, "0" if there is none
%   
%   Copyright (c) 2020, Ian G. Bennett
%   All rights reserved.
%   Development funded by the University of Toronto, Department of
%   Mechanical and Industrial Engineering.
%   Distributed under GNU AGPLv3 license.

% Determine if all robot points are still inside the maze
collision1 = ~inpolygon(bot_pos(:,1), bot_pos(:,2), maze(:,1), maze(:,2));

% Determine if any maze points are within the robot
collision2 = inpolygon(maze(:,1), maze(:,2), bot_pos(:,1), bot_pos(:,2));

if (any(collision2) || any(collision1))
    collision = 1;
else
    collision = 0;
end

end

