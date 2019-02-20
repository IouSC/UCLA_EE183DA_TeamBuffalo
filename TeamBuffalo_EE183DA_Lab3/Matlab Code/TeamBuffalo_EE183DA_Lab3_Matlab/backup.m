function [command,val] = policy(policy_p,lambda,pe,rows,action) 
    % Initialization of initial policy \pi_0
    command = policy_p;
    exit = 0;
    
    while exit == 0 
        % Initialization
        val = init();
     
        value_difference = 10e5; 
        
        % Policy Evaluation
        while value_difference > 10e-5
            % Store previous value to temp
            temp = val; 
            % Compute and update new value and store
            for x = 1:6
                for y = 1:6
                    for h = 1:12
                        % Find all possible movements due to prerotation
                        % Note: here the next state and doesn't matter
                        [garbage,s] = probability(pe,[x;y;h],command((rows+1)-y,x,h,:),[1,1,1]');
                        % Update the value calculated for policy before update
                        val((rows+1)-y,x,h) = s(4,1) * (reward([s(1,1);s(2,1)]) + lambda*val((rows+1)-s(2,1),s(1,1),s(3,1))) +...
                                              s(4,2) * (reward([s(1,2);s(2,2)]) + lambda*val((rows+1)-s(2,2),s(1,2),s(3,2))) +...
                                              s(4,3) * (reward([s(1,3);s(2,3)]) + lambda*val((rows+1)-s(2,3),s(1,3),s(3,3)));
                    end
                end
            end
            % Compare new value with previous value
            value_difference = sum(sum(sum(abs(temp-val))));
        end 
        
        % Policy Improvement
        command_copy = command; 
        for x = 1:6
            for y = 1:6
                for h = 1:12
                    set = []; 
                    for j = 1:6
                    % Find all possible movements due to prerotation
                    % Note: here the next state and doesn't matter
                    [garbage,s] = probability(pe,[x;y;h],action(j,:),[1,1,1]');
                    % Store the value calculated for each actions
                    % Stored in a temp array
                    set(j) = s(4,1) * (reward([s(1,1);s(2,1)]) + lambda*val((rows+1)-s(2,1),s(1,1),s(3,1))) +...
                             s(4,2) * (reward([s(1,2);s(2,2)]) + lambda*val((rows+1)-s(2,2),s(1,2),s(3,2))) +...
                             s(4,3) * (reward([s(1,3);s(2,3)]) + lambda*val((rows+1)-s(2,3),s(1,3),s(3,3)));
                    end
                    % Special case
                    % When we reach the goal (green) square
                    % Robot can perform action (0,0) --> stay still (no movement, no rotation)
                    if (x == 5 && y == 5)
                        [garbage,s] = probability(pe,[x;y;h],action(7,:),[1,1,1]');
                        set(7) = s(4,1) * (reward([s(1,1);s(2,1)]) + lambda*val((rows+1)-s(2,1),s(1,1),s(3,1))) +...
                                 s(4,2) * (reward([s(1,2);s(2,2)]) + lambda*val((rows+1)-s(2,2),s(1,2),s(3,2))) +...
                                 s(4,3) * (reward([s(1,3);s(2,3)]) + lambda*val((rows+1)-s(2,3),s(1,3),s(3,3)));
                    end 
                    % Find the best action (maximum value) and store it
                    [M,I] = max(set);
                    % Find value of original action
                    [garbage,orig] = probability(pe,[x;y;h],command((rows+1)-y,x,h,:),[1,1,1]');
                    temp = orig(4,1) * (reward([orig(1,1);orig(2,1)]) + lambda*val((rows+1)-orig(2,1),orig(1,1),orig(3,1))) +...
                           orig(4,2) * (reward([orig(1,2);orig(2,2)]) + lambda*val((rows+1)-orig(2,2),orig(1,2),orig(3,2))) +...
                           orig(4,3) * (reward([orig(1,3);orig(2,3)]) + lambda*val((rows+1)-orig(2,3),orig(1,3),orig(3,3)));
                    % Store new policy if find better
                    if temp < M
                        % Store movement (forward/backward) into command
                        command((rows+1)-y,x,h,1) = action(I,1);
                        % Store rotation (left/none/right) into command
                        command((rows+1)-y,x,h,2) = action(I,2);
                    end 
                end
            end
        end
        
        % If policy converges, exit the while loop and set to true (1)
        if isequal(command_copy,command) == 0
            exit = 0; 
        else 
            exit = 1;
        end 
    end
     
end