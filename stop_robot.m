function [  ] = stop_robot( )
%This function stops the robot and completely ends the program.

global fire;
global flag_change;

if flag_change
    disp('Robot stop controller engaged');
    flag_change = false;
    
arrobot_stop
arrobot_disconnect

fire = 0;
disp('System exit')

end

