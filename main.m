%The complete hybrid automaton consisting of three stages.
%Refer notes for diagram

%This module just initialises everything,
%declares the global variables
%and the calls the gtg controller

clc;
clear all;
close all;

%Initialise
aria_init
arrobot_connect

h_main = figure('name','Main');
axis([-1500 10000 -5000 6000]);
hold on;

%Display robot position 
%Initial is always zero as per ARIA
hr = displayrobo();

%Define goal (x less than 4854,y less than 4037)
global goal
goal = [6000;0.0];

%Display goal
scatter(goal(1),goal(2),100,'filled','green');
drawnow;

%The prograss variable initialisation
global dprogress %in mm
dprogress = -1;

%Whether or not to run
global fire
fire = 1;

delete(hr);
%It all begins at the Go to goal controller (gtg).
guard(1)
global flag_change
flag_change = true;

global map;
map = [];

%Begin the controller loop
%Till there burns the fire
while fire
    
    if fire == 1
        gtg()
        continue
    elseif fire == 2
        ao_gtg()
        continue         
    elseif fire == 3
        ao()    
        continue    
    elseif fire == 41
        fw('l') 
        continue               
    elseif fire == 42
        fw('r')
        continue
    else
        stop_robot()
    end
end

