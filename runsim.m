close all;
clear;

addpath('utils');

%% Trajectory generation

 trajhandle = @traj_generator2;
 

%   waypoints = [0      0      0;
%                0      0       30
%                10      0      80;
%                10      0      118.5;
%                10      0      80;
%                10   -33.34    60.75;
%                10      0      80;
%                10    33.34    60.75;
%                10      0      80
%                0       0      30
%                0       0       0]';
%                
waypoints = [ 0 0 0;
              1 1 1;
              3 1 3;
              1 1 1];
%   waypoints = [0      0      80;
%                0      0      118.5;
%                0      0      80;
%                0   -33.34    60.75;
%                0      0      80;
%                0    33.34     60.75;
%                0      0      80]';

           
          
trajhandle([],[],waypoints);

controlhandle = @controller;

[t, state] = simulation_3d(trajhandle, controlhandle);
