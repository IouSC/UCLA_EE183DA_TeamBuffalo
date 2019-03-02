%% EE183DA Lab 4
    % Team Buffalo

%% Main
clc; clear all; close all;

% Calculate Computational Time
% Using MATLAB built in tic-toc function
tic;

% Variables and Constants
L = 762.5; % Length of the box [mm]
W = 497.5; % Width of the box [mm]

% Plot Obstacle in the box
obstacle1 = [600,0,L-600,100];
obstacle2 = [600,300,L-600,100];
obstacle3 = [250,200,100,100];
rectangle('Position',obstacle1,'FaceColor',[0.2, 0.8, 0.8]);
rectangle('Position',obstacle2,'FaceColor',[0.2, 0.8, 0.8]);
rectangle('Position',obstacle3,'FaceColor',[0.2, 0.8, 0.8]);
text(630,45,'Obstacle 1','interpreter','latex');
text(630,345,'Obstacle 2','interpreter','latex');
text(260,250,'Obstacle 3','interpreter','latex');

% Create a figure to plot the state of the car
figure(1);
hold on;

axis([0 L 0 W]);
box on; tightfig;
robot_path_dis = 0; 
[x,robot_path_dis] = milestone1([150 150 270]'); 
fprintf('------------------------Milestone1 Completed------------------------\n');
x = milestone2(x,robot_path_dis); 
fprintf('------------------------Milestone2 Completed------------------------\n');
milestone3(x,robot_path_dis);
fprintf('------------------------Milestone3 Completed------------------------\n');

computationalTime = toc;
computationalTimeStr = ['Computational Time = ',num2str(computationalTime),' [s]'];
fprintf(computationalTimeStr);