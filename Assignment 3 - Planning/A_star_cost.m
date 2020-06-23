%% FORMALITIES
% Author: Alexander Wallén Kiessling
% Date: 23 June 2020
% Description: Testing the cost function of A* algorithm and plotting it
% to see if results match expectations.

%% SETUP

% Clear Workspace
clc;
clear all;
close all;

% Declarations
x0 = 0; %initial x-coordinate
y0 = 2; %initial y-coordinate
xt = 20; %target x-coordinate
yt = 8; %target y-coordinate

obstacleX = [11.635, 8.50, 13.23, 4.8847, 14.44, 11.843, 8.567, 5.821, 13.82, 6.81, 5.98, 14.62, 10.235, 8.896, 4.955, 15.47, 6.364, 10.45];
obstacleY = [8.162, 1.712, 5.153, 6.555, 7.457, 2.588, 4.31, 2.984, 0.7336, 7.9937, 0.713, 2.895, 6.416, 9.02, 9.42, 5.023, 5.032, 0.692];
obstacleR = [0.631, 0.612, 0.537, 0.569, 0.748, 0.723, 0.7877, 0.5051, 0.5067, 0.59714, 0.5912, 0.6606, 0.55333, 0.6741, 0.5176, 0.522, 0.5, 0.501];
%% FUNCTION DEFINITIONS

cost = @(x,y) sqrt((x0 - x).^2 + (y0 - y).^2) + sqrt((xt - x).^2 + (yt - y).^2);

%% DRIVER CODE

figure(1);
plot([0 20], [2 8])
hold on
axis([0 20 0 20])
axis manual
for i = 1:18
    centers = [obstacleX(i) obstacleY(i)];
    viscircles(centers, obstacleR(i));
end

figure(2);
fmesh(cost,[0,20]);
hold off

%note: optimal cost is 20.8806, occurring at start and finish, and in
%straight line inbetween.
%disp(cost(0,2))
%disp(cost(20,8))
