function [ ] = fw( side )
%The Follow wall controller

global prev_heading_error
global total_heading_error
global flag_change

if flag_change
        disp('Follow wall controller engaged on'); side
        flag_change = false;
end


velMult = 75; %mm/s

%Controller parameters
Kp = 0.2;
Kd = 0.4;
Ki = 0.0;

%Define the robot parameters
Rbtw = arrobot_width;

dfw = 2* Rbtw;%Follow wall threshold in mm

%Setting constant velocity
arrobot_setvel(velMult);
  
hfw(1) = displayrobo();
 
%get current robot position from aria
[Xa,Ya,thetaa] = localise();
  
%Get sonar readings
sonoffset = [-90 -50 -30 -10 10 30 50 90];
  
r = zeros(4,1);
readings = zeros(2,4);
  
if(side == 'l')
    for i = 1:4  
        r(i) = arrobot_getsonarrange(i-1);
        if(r(i) > 2000)
            r(i) = 2000;    
        end
        angl = ((thetaa - sonoffset(i))/180)*pi;
        delx = r(i)*cos(angl);
        dely = r(i)*sin(angl);
        readings(1,i) = delx;
        readings(2,i) = dely;
    end
else
    for i = 1:4
        r(i) = arrobot_getsonarrange(8-i);
        if(r(i) > 2000)
            r(i) = 2000;
        end
        angl = ((thetaa - sonoffset(9-i))/180)*pi;
        delx = r(i)*cos(angl);
        dely = r(i)*sin(angl);
        readings(1,i) = delx;
        readings(2,i) = dely;
    end
end

[~,Ix] = sort(r);

if(Ix(1) > Ix(2))
    temp = Ix(1);
    Ix(1) = Ix(2);
    Ix(2) = temp;
end

Ufw_t = readings(:,Ix(2)) - readings(:,Ix(1));

Upfw_t = Ufw_t/norm(Ufw_t);
hfw(2) = line([(readings(1,Ix(1))+Xa),(readings(1,Ix(2))+Xa)],[(readings(2,Ix(1))+Ya),(readings(2,Ix(2))+Ya)],'color','green');
drawnow;

temp = (dot(readings(:,Ix(1)),Upfw_t))*Upfw_t;
Ufw_p = readings(:,Ix(1)) - temp;

Upfw_p = Ufw_p - (dfw*(Ufw_p/norm(Ufw_p)));
hfw(3) = line([Xa,Ufw_p(1,1)+Xa],[Ya,Ufw_p(2,1)+Ya],'color','red');
drawnow;

Upfw_p = Upfw_p/norm(Upfw_p);

Ufw  = ((Upfw_p) + (4*Upfw_t))/5;
%Determine how far to rotate to follow wall
dt =  (atan2(Ufw(2,1),Ufw(1,1)) * (180/3.14159)) - thetaa;
dt = mod((dt + 180), 360) - 180; % restrict to (-180,180);
Ufw = dfw * Ufw;
hfw(4) = line([Xa,Ufw(1,1)+Xa],[Ya,Ufw(2,1)+Ya],'color','blue');
drawnow;
  
%request robot to turn
W = (Kp*dt) + (Ki*total_heading_error) + (Kd*(dt - prev_heading_error));
arrobot_setrotvel(W)

total_heading_error = total_heading_error + dt;
prev_heading_error = dt;

delete(hfw(:));
  
%Check events
if(at_goal())
    %call stop_robot();
    guard(5);
elseif((~no_progress())&&(~chk_wall(side)))
    %call ao_gtg();
    guard(2);
end
  
end


