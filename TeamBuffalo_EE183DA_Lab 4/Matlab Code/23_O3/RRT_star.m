function[x_next, y_next] = RRT_star(x, y)
    % Variables and Constants 
    L = 762;        % Length of the box [mm]
    W = 497;        % Width of the box [mm]
    buffer = 80;    % Buffer added to box and obstacles [mm]
    EPS =75;        % Maximum step size [mm]
    exit = 0;

    % Define Obstacles
    obstacle1 = [600,0,L-600,100];
    obstacle2 = [600,300,L-600,100];
    obstacle3 = [250,0,100,250];
    obstacle1 = obstacle1 + [-buffer,-buffer,2*buffer,2*buffer];
    obstacle2 = obstacle2 + [-buffer,-buffer,2*buffer,2*buffer];
    obstacle3 = obstacle3 + [-buffer,-buffer,2*buffer,2*buffer];
    
    % Initial state
    q_start.coord = [x y];
    q_start.cost = 0;
    q_start.parent = 0;
    % Final state
    q_goal.coord = [525 200];
    q_goal.cost = 0;
    nodes(1) = q_start;

    while exit==0
        for i = 1:1000
            i
            % Sample(x)    
            q_rand = [randi(L-160)+80 randi(W-120)+80];

            % Nearest
            [q_near M I] = nearest(nodes,q_rand);

            % Drive
            q_new.coord = steer(q_rand, q_near.coord, M, EPS);

            % If no Collision then
            if  noCollision(q_rand, q_near.coord, obstacle1) && noCollision(q_rand, q_near.coord, obstacle3) && noCollision(q_rand, q_near.coord, obstacle2)
                q_new.cost = pdist([q_new.coord; q_near.coord]) + q_near.cost;

                % Update Nearest
                q_nearest = [];
                r = EPS;
                neighbor_count = 1;
                for j = 1:1:length(nodes)
                    if  noCollision(nodes(j).coord, q_new.coord, obstacle1) && noCollision(q_rand, q_near.coord, obstacle3) && noCollision(nodes(j).coord, q_new.coord, obstacle2) && pdist([nodes(j).coord; q_new.coord]) < r
                        q_nearest(neighbor_count).coord = nodes(j).coord;
                        q_nearest(neighbor_count).cost = nodes(j).cost;
                        neighbor_count = neighbor_count+1;
                    end
                end

                % Select Parent
                q_min = q_near;
                C_min = q_new.cost;

                % Review
                for k = 1:length(q_nearest)
                    if  noCollision(q_nearest(k).coord, q_new.coord, obstacle1) && noCollision(q_rand, q_near.coord, obstacle3) && noCollision(q_nearest(k).coord, q_new.coord, obstacle2) && q_nearest(k).cost + pdist([q_nearest(k).coord; q_new.coord]) < C_min
                        q_min = q_nearest(k);
                        C_min = q_nearest(k).cost + pdist([q_nearest(k).coord; q_new.coord]);
                    end
                end

                for j = 1:length(nodes)
                    if nodes(j).coord == q_min.coord
                        q_new.parent = j;
                        q.new.cost =C_min; 
                    end
                end

                % Insert node
                nodes = [nodes q_new];
           end
       end
       
       % Check we have find final goal or not
       for j = 1:length(nodes)
           if noCollision(nodes(j).coord, q_goal.coord, obstacle1) && noCollision(q_rand, q_near.coord, obstacle3) && noCollision(nodes(j).coord, q_goal.coord, obstacle2) && pdist([nodes(j).coord; q_goal.coord]) < EPS
               q_rand = q_goal.coord;
               exit = 1; 
               break
           end
       end
       % If not, repeat what we did pervious for 1000 points more
       % If yes, add that point to our node set (also perform collision check)
       if exit ==1 
            % Nearest
            [q_near M I] = nearest(nodes,q_rand);

            % Drive
            q_new.coord = steer(q_rand, q_near.coord, M, EPS);

            % If no Collision then
            if  noCollision(q_rand, q_near.coord, obstacle1) && noCollision(q_rand, q_near.coord, obstacle3) && noCollision(q_rand, q_near.coord, obstacle2)
                q_new.cost = pdist([q_new.coord; q_near.coord]) + q_near.cost;

                % Update Nearest
                q_nearest = [];
                r = EPS;
                neighbor_count = 1;
                for j = 1:1:length(nodes)
                    if  noCollision(nodes(j).coord, q_new.coord, obstacle1) && noCollision(q_rand, q_near.coord, obstacle3) && noCollision(nodes(j).coord, q_new.coord, obstacle2) && pdist([nodes(j).coord; q_new.coord]) < r
                        q_nearest(neighbor_count).coord = nodes(j).coord;
                        q_nearest(neighbor_count).cost = nodes(j).cost;
                        neighbor_count = neighbor_count+1;
                    end
                end

                % Select Parent
                q_min = q_near;
                C_min = q_new.cost;

                % Review
                for k = 1:length(q_nearest)
                    if  noCollision(q_nearest(k).coord, q_new.coord, obstacle1) && noCollision(q_rand, q_near.coord, obstacle3) && noCollision(q_nearest(k).coord, q_new.coord, obstacle2) && q_nearest(k).cost + pdist([q_nearest(k).coord; q_new.coord]) < C_min
                        q_min = q_nearest(k);
                        C_min = q_nearest(k).cost + pdist([q_nearest(k).coord; q_new.coord]);
                    end
                end

                for j = 1:length(nodes)
                    if nodes(j).coord == q_min.coord
                        q_new.parent = j;
                        q.new.cost =C_min;
                    end
                end

                %Insert node
                nodes = [nodes q_new];
            end
        end
    end 

    q_end = nodes(length(nodes));
    while nodes(q_end.parent).parent ~= 0
        start = q_end.parent;
        q_end = nodes(start);
    end
    
    x_next = q_end.coord(1);
    y_next = q_end.coord(2);
    path = line([x_next,q_start.coord(1)],[y_next,q_start.coord(2)],'Color','m','LineWidth',2);
    path.Annotation.LegendInformation.IconDisplayStyle = 'off';
end 
       
   
