function [ flag ] = at_goal()
%Event which checks if we have reached the goal (within threshold)

distThresh = 150; %mm

global goal;
xd = goal(1,1);
yd = goal(2,1);

%Define the robot parameters
Rbtl = arrobot_length;
Rbtw = arrobot_width;

% get current robot position from aria
[xa,ya,dummy] = localise();
xa = xa + Rbtl/2;
ya = ya + Rbtw/2;

% Check if we've reached goal point
d = sqrt( (xd - xa)^2 + (yd - ya)^2 );
%fprintf(' distance remaining: %f\n', d);

if d <= distThresh
    disp 'reached point';
    flag  = 1;
    %disp('At goal event flaged');
else 
    flag  = 0;
end

end

