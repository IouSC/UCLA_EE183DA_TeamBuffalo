function [val,command] = value(V,action,lambda,pe,rows) 
    % ===== Initialization ===== %
    val = V; 
    command = [];
    
    % ===== ?ompute the value ===== %
    for x = 1:6
        for y = 1:6
            % Iterate through 12 possible headings
            for h = 1:12
                % Iterate through 7 possible actions
                value_allActions = []; 
                for j = 1:6
                    % Find all possible movements due to prerotation
                    % Note: here the next state and doesn't matter
                    [garbage,s] = probability(pe,[x;y;h],action(j,:),[1,1,1]');
                    % Store the value calculated for each actions
                    % Stored in a temp array
                    value_allActions(j) = s(4,1) * (reward([s(1,1);s(2,1)]) + lambda*val((rows+1)-s(2,1),s(1,1),s(3,1))) +...
                                          s(4,2) * (reward([s(1,2);s(2,2)]) + lambda*val((rows+1)-s(2,2),s(1,2),s(3,2))) +...
                                          s(4,3) * (reward([s(1,3);s(2,3)]) + lambda*val((rows+1)-s(2,3),s(1,3),s(3,3)));
                end
                % Special case
                % When we reach the goal (green) square
                % Robot can perform action (0,0) --> stay still (no movement, no rotation)
                if (x == 5 && y == 5)
                    [garbage,s] = probability(pe,[x;y;h],action(7,:),[1,1,1]');
                    value_allActions(7) = s(4,1) * (reward([s(1,1);s(2,1)]) + lambda*val((rows+1)-s(2,1),s(1,1),s(3,1))) +...
                                          s(4,2) * (reward([s(1,2);s(2,2)]) + lambda*val((rows+1)-s(2,2),s(1,2),s(3,2))) +...
                                          s(4,3) * (reward([s(1,3);s(2,3)]) + lambda*val((rows+1)-s(2,3),s(1,3),s(3,3)));
                end 
                % Find the best action (maximum value) and store it
                [M,I] = max(value_allActions); 
                val((rows+1)-y,x,h)= M; 
                % Store movement (forward/backward) into command
                command((rows+1)-y,x,h,1) = action(I,1);
                % Store rotation (left/none/right) into command
                command((rows+1)-y,x,h,2) = action(I,2);
            end 
        end 
    end 
end