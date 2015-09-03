function [Rx,Ry,Rth] = localise(  )

%function which localises the robot
%The x and y locations and orientation is returned.

[Rx,Ry,Rth] = arrobot_getpose;
% pause(2);
% while([Rx,Ry,Rth] ~= arrobot_getpose)
%     pause(4);
%     display 'wait performing move';
%     [Rx,Ry,Rth] = arrobot_getpose;
% end
% [Rx,Ry,Rth] = arrobot_getpose;
end

