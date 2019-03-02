%% EE183DA Lab 4
    % Team Buffalo
    % RRT Algorithm
    % Adapted from: Sai Vemprala [See report bibiography page]

%% RRT Algorithm
clc; close all; clear all;

% Variables and Constants 
L = 762;      % Length of the box [mm]
W = 497;      % Width of the box [mm]
buffer = 80;    % Border added to box and obstacles [mm]
EPS =30;        % Maximum step size [mm]
exit = 0;

% Define Obstacles
obstacle1 = [600,0,L-600,100];
obstacle2 = [600,300,L-600,100];
  
% Initial state
q_start.coord = [100 100];
q_start.parent = 0;
% Final state
q_goal.coord = [600 190];
nodes(1) = q_start;

% Create a figure to plot route
figure(1);
hold on;
% Set figure
title('$RRT$ Algorithm','interpreter','latex');
xlabel('$p_x$','interpreter','latex');
ylabel('$p_y$','interpreter','latex');
set(gca,'ticklabelinterpreter','latex')
axis([0 L 0 W]);
box on; tightfig;

% Plot Initial state and Final state
plot(q_start.coord(1), q_start.coord(2),'bo')
plot(q_goal.coord(1),q_goal.coord(2),'bo');

% Plot Obstacle in the box
rectangle('Position',obstacle1,'FaceColor',[0.2, 0.8, 0.8]);
rectangle('Position',obstacle2,'FaceColor',[0.2, 0.8, 0.8]);
text(630,45,'Obstacle 1','interpreter','latex');
text(630,345,'Obstacle 2','interpreter','latex');

% Draw allowable region given border from each obstacles and box
border1 = [buffer,buffer,L-2*buffer,W-2*buffer];
border2 = obstacle1 + [-buffer,-buffer,2*buffer,2*buffer];
border3 = obstacle2 + [-buffer,-buffer,2*buffer,2*buffer];
rectangle('Position',border1,'EdgeColor','r');
rectangle('Position',border2,'EdgeColor','r');
rectangle('Position',border3,'EdgeColor','r');

% Adding boundry to obstacle
obstacle1 = border2; 
obstacle2 = border3;

% Using MATLAB built in tic-toc function to compute computational time
tic;
while exit==0
    % Sample(x) 
    q_rand = [randi(L-160)+80 randi(W-160)+80];
    % Break if goal state is reached
    for j = 1:length(nodes)
        if noCollision(nodes(j).coord,q_goal.coord,obstacle1) &&...
           noCollision(nodes(j).coord,q_goal.coord,obstacle2) &&...
           pdist([nodes(j).coord;q_goal.coord]) < EPS
            q_rand = q_goal.coord;
            exit = 1; 
            break
        end
    end
    plot(q_rand(1),q_rand(2),'x','Color','g');
    
    % Nearest
    [q_near M I] = nearest(nodes,q_rand);
    
    % Drive
    q_new.coord = steer(q_rand,q_near.coord,M,EPS);
    
    % Check Collision
    % If no collision then...
    if  noCollision(q_rand,q_near.coord,obstacle1) && noCollision(q_rand,q_near.coord,obstacle2)
        % Create a stright line between the two points
        % To determine how to move the car / what inputs to give it
        line([q_near.coord(1),q_new.coord(1)], [q_near.coord(2),q_new.coord(2)],'Color','k');
        drawnow;       
        % InsertNode 
        q_new.parent = I;
        nodes = [nodes q_new];
    end
end
computationalTime = toc;

% Show the RRT path
q_end = nodes(length(nodes));
while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2),nodes(start).coord(2)],'Color','r','LineWidth', 2);
    q_end = nodes(start);
end

% Calculate total distance traveled by the robot;
annotationStr = ['$RRT$ Computational Time = ' num2str(computationalTime) ' [s]']; 
annotation('textbox',[.1 .875 .5 .05],'String',annotationStr,'interpreter','latex',...
           'FitBoxToText','on','EdgeColor','none');

% Save final result figure to .png
savetitleStr = ['TeamBuffalo_EE183DA_Lab4_RRT.png'];
saveas(gcf,savetitleStr);
hold off;