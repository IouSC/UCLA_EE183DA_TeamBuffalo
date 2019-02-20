function [val] = get_value(command,lambda,pe,rows,reward)
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
                    val((rows+1)-y,x,h) = s(4,1) * (reward((rows+1)-s(2,1),s(1,1),s(3,1)) + lambda*val((rows+1)-s(2,1),s(1,1),s(3,1))) +...
                                          s(4,2) * (reward((rows+1)-s(2,2),s(1,2),s(3,2)) + lambda*val((rows+1)-s(2,2),s(1,2),s(3,2))) +...
                                          s(4,3) * (reward((rows+1)-s(2,3),s(1,3),s(3,3)) + lambda*val((rows+1)-s(2,3),s(1,3),s(3,3)));
                end 
            end
        end
        % Compare new value with previous value
        value_difference = sum(sum(sum(abs(temp-val))));
    end 
end