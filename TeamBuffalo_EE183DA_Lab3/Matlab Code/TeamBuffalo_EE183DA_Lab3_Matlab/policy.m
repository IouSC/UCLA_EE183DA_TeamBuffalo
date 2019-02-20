function [command,val] = policy(policy_p,lambda,pe,rows,action) 
    % Initialization of initial policy \pi_0
    command = policy_p;
    exit = 0;
    
    while exit == 0 
        command_copy = command; 
        % Initialization
        val = get_value(command,lambda,pe,rows);
        command = get_policy(command_copy,val,lambda,pe,rows,action);
        % Policy Improvement
        
        
        % If policy converges, exit the while loop and set to true (1)
        if isequal(command_copy,command) == 0
            exit = 0; 
        else 
            exit = 1;
        end 
    end
    val = get_value(command,lambda,pe,rows);
end