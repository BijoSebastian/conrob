function [ ] = gtg( )
%The Go to goal controller

global goal
global prev_heading_error
global total_heading_error   
global flag_change

if flag_change
    disp('Go to goal controller engaged');
    flag_change = false;
end

velMult = 100; %mm/s
distThresh = 150; %mm

%Controller parameters
Kp = 0.5;
Kd = 1.0;
Ki = 0.0;

xd = goal(1,1);
yd = goal(2,1);

%Define the robot parameters
Rbtl = arrobot_length;
Rbtw = arrobot_width;

hgtg(1) = displayrobo();

%get current robot position from aria
[xa,ya,thetaa] = localise();
xa = xa + Rbtl/2;
ya = ya + Rbtw/2;

%Determine how far to rotate to face goal point
dt =  (atan2(yd - ya, xd - xa) * (180/3.14159)) - thetaa;
dt = mod((dt + 180), 360) - 180; % restrict to (-180,180);
hgtg(2) = line([xa,xa+(1000*(cos(deg2rad(dt + thetaa))))],[ya,ya+(1000*(sin(deg2rad(dt + thetaa))))],'color','green');
drawnow;
%fprintf(' angle remaining: %f\n', dt);

%request robot to turn
W = (Kp*dt) + (Ki*total_heading_error) + (Kd*(dt - prev_heading_error));
arrobot_setrotvel(W);
total_heading_error = total_heading_error + dt;
prev_heading_error = dt;

%Calculate distance to goal
d = sqrt( (xd - xa)^2 + (yd - ya)^2 );
%fprintf(' distance remaining: %f\n', d);

%request robot velocity proportionally to distance remaining
vel = ((atan((d - distThresh)/25)) - (atan(dt/10)))*velMult;
arrobot_setvel(vel);

pause(0.08);

delete(hgtg(:));

%Check events
if(at_goal())
    %call stop_robot();
    guard(5);
elseif(at_obstacle())
    %call ao_gtg();
    guard(2);
elseif(no_progress())
    if(chk_wall('l'))
        %call fw('l');
        guard(41);
    else
        %call fw('r');
        guard(42);
    end
end

end
  


