function [ h ] = displayrobo(  )

%Function to display the robot and the goal 
% Returns the handle of printing the robot

%Define the robot parameters
Rbtl = arrobot_length;
Rbtw = arrobot_width;

[Rx,Ry,Rth] = localise();
Rx = Rx + Rbtl/2;
Ry = Ry +Rbtw/2;

h = scatter(Rx,Ry,100,'filled','red');




