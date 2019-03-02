%% EE183DA Lab 4
    % Team Buffalo
    % RRT* Algorithm
    % Adapted from: Sai Vemprala [See report bibiography page]

%% RRT* Algorithm
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
q_start.cost = 0;
q_start.parent = 0;
% Final state
q_goal.coord = [600 190];
q_goal.cost = 0;
nodes(1) = q_start;

% Create a figure to plot route
figure(1);
hold on;
% Set figure
title('$RRT*$ Algorithm','interpreter','latex');
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
    for i = 1:2000
        % Sample(x)     
        q_rand = [randi(L-160)+80 randi(W-160)+80];
        plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410])

        % Nearest
        dist_set = [];
        for j = 1:length(nodes)
            n = nodes(j);
            tmp = pdist([n.coord; q_rand]);
            dist_set(j) = tmp;
        end
        [M, I] = min(dist_set);
        q_near = nodes(I);

        % Drive
        q_new.coord = steer(q_rand, q_near.coord, M, EPS);

        %if no Collision then
        if  noCollision(q_rand, q_near.coord, obstacle1) && noCollision(q_rand, q_near.coord, obstacle2)
            line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 1);
            drawnow
            hold on
            q_new.cost = pdist([q_new.coord; q_near.coord]) + q_near.cost;

            % Near
            q_nearest = [];
            neighbor_count = 1;
            for j = 1:1:length(nodes)
                if  noCollision(nodes(j).coord, q_new.coord, obstacle1) && noCollision(nodes(j).coord, q_new.coord, obstacle2) && pdist([nodes(j).coord; q_new.coord]) < EPS
                    q_nearest(neighbor_count).coord = nodes(j).coord;
                    q_nearest(neighbor_count).cost = nodes(j).cost;
                    neighbor_count = neighbor_count+1;
                end
            end

            % Select Parent
            q_min = q_near;
            C_min = q_new.cost;
            
            %Review
            for k = 1:length(q_nearest)
                if  noCollision(q_nearest(k).coord, q_new.coord, obstacle1) && noCollision(q_nearest(k).coord, q_new.coord, obstacle2) && q_nearest(k).cost + pdist([q_nearest(k).coord; q_new.coord]) < C_min
                    q_min = q_nearest(k);
                    %Insert Edge
                    C_min = q_nearest(k).cost + pdist([q_nearest(k).coord; q_new.coord]);
                    line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');                
                    hold on
                end
            end
            for j = 1:length(nodes)
                if nodes(j).coord == q_min.coord
                    q_new.parent = j;
                end
            end

            % Insert node 
            nodes = [nodes q_new];
        end
    end
    
    % Check we have find final goal or not
    % If not, repeat what we did pervious
    % if yes, add that point to our node set
    for j = 1:length(nodes)
        if noCollision(nodes(j).coord, q_goal.coord, obstacle1) && noCollision(nodes(j).coord, q_goal.coord, obstacle2) && pdist([nodes(j).coord; q_goal.coord]) < EPS
            q_rand = q_goal.coord;
            exit = 1; 
            break
        end
    end
    if exit == 1
        plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410])

        dist_set = [];
        for j = 1:length(nodes)
            n = nodes(j);
            tmp = pdist([n.coord; q_rand]);
            dist_set(j) = tmp;
        end
        [M, I] = min(dist_set);
        q_near = nodes(I);

        q_new.coord = steer(q_rand, q_near.coord, M, EPS);
        if  noCollision(q_rand, q_near.coord, obstacle1) && noCollision(q_rand, q_near.coord, obstacle2)
            line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 1);
            drawnow
            hold on
            q_new.cost = pdist([q_new.coord; q_near.coord]) + q_near.cost;

            q_nearest = [];
            neighbor_count = 1;
            for j = 1:1:length(nodes)
                if  noCollision(nodes(j).coord, q_new.coord, obstacle1) && noCollision(nodes(j).coord, q_new.coord, obstacle2) && pdist([nodes(j).coord; q_new.coord]) < EPS
                    q_nearest(neighbor_count).coord = nodes(j).coord;
                    q_nearest(neighbor_count).cost = nodes(j).cost;
                    neighbor_count = neighbor_count+1;
                end
            end

            q_min = q_near;
            C_min = q_new.cost;

            for k = 1:length(q_nearest)
                if  noCollision(q_nearest(k).coord, q_new.coord, obstacle1) && noCollision(q_nearest(k).coord, q_new.coord, obstacle2) && q_nearest(k).cost + pdist([q_nearest(k).coord; q_new.coord]) < C_min
                    q_min = q_nearest(k);
                    C_min = q_nearest(k).cost + pdist([q_nearest(k).coord; q_new.coord]);
                    line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');                
                    hold on
                end
            end

            for j = 1:length(nodes)
                if nodes(j).coord == q_min.coord
                    q_new.parent = j;
                end
            end        
            nodes = [nodes q_new];
        end
    end 
end 
computationalTime = toc;

%show the edge we find from initial to final
q_end = nodes(length(nodes));
while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    hold on
    q_end = nodes(start);
end

% Calculate total distance traveled by the robot;
annotationStr = ['$RRT*$ Computational Time = ' num2str(computationalTime) ' [s]']; 
annotation('textbox',[.1 .875 .5 .05],'String',annotationStr,'interpreter','latex',...
           'FitBoxToText','on','EdgeColor','none');

% Save final result figure to .png
savetitleStr = ['TeamBuffalo_EE183DA_Lab4_RRT_star.png'];
saveas(gcf,savetitleStr);
hold off;