function [ flag ] = no_progress()
%Event which checks progress
%If made ==> sets it.
%If not ==> checks it and raises flag

global goal;
xd = goal(1,1);
yd = goal(2,1);

global dprogress

%Define the robot parameters
Rbtl = arrobot_length;
Rbtw = arrobot_width;

%Epsilon slack
E = Rbtw/2.0; 

% get current robot position from aria
[xa,ya,~] = localise();
xa = xa + Rbtl/2;
ya = ya + Rbtw/2;

dist = sqrt( (xd - xa)^2 + (yd - ya)^2 );

%The first setting
if(dprogress < 0)
    dprogress = dist; 
    flag  = 0;
    return 
end

if(dprogress > dist)
    %Progress made
    dprogress = dist;
    flag = 0;
    return
elseif(dist > (dprogress + E))
    %Progress not being made
    %disp('no progress event flagged');
    flag = 1;
    return
else
    %Within slack
    flag = 0;
end

end

