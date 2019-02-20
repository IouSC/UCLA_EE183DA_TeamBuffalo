function [prob,state_p] = probability(pe,state_c,action,state_n)
    % ===== Heading ===== %
    % circular loop to account for the flaw in mod 12
    heading = [12 ,1 ,2, 3, 4, 5, 6, 7, 8, 9, 10, 11 ,12, 1];
    
    % ===== Possible States ===== %
    % state_p is a R^{2} 
    % cause there are three possible prerotation outcome
    state_p = [state_c  state_c  state_c];
    if (pe ~= 0 && action(1) ~=0)
        % prerotation -1 (left) 
        state_p(3,1) = heading(state_p(3,1));
        % prerotation 0 (no prerotation)
        state_p(3,2) = heading(state_p(3,2)+1);
        % prerotation 1 (right)
        state_p(3,3) = heading(state_p(3,3)+2);    
    end
   
    % ===== Update Possible Next State ===== %
    % State depedning on heading
    for i =1:3
        % (x,y)
        % If statements account for when robot is at the border
        % when @ the border, robot can still rotate but cannot perform movement 
        switch state_p(3,i)
            case {11,12,1}
                state_p(2,i) = state_p(2,i) + action(1); 
                if (state_p(2,i) == 7 || state_p(2,i) == 0)
                    state_p(2,i) = state_c(2);
                end 
            case {2,3,4}
                state_p(1,i) = state_p(1,i) + action(1);
                if (state_p(1,i) == 7 || state_p(1,i) == 0)
                    state_p(1,i) = state_c(1);
                end 
            case {5,6,7}
                state_p(2,i) = state_p(2,i) - action(1); 
                if (state_p(2,i) == 0 || state_p(2,i) == 7)
                    state_p(2,i) = state_c(2);
                end 
            case {8,9,10}
                state_p(1,i) = state_p(1,i) - action(1); 
                if (state_p(1,i) == 0 || state_p(1,i) == 7)
                    state_p(1,i) = state_c(1);
                end 
        end 
        % Heading
        state_p(3,i) = heading((state_p(3,i)+1) + action(2));       
        
    end
    % Probability
    % Attach the probability corresponding to each possible states
    % For prerotated (-1/1) states: pe
    % For non-prerotated (0) states: 1-2*pe
    state_p(4,:) = [pe, 1-2*pe, pe];

    % ===== Finding probability for the selected state ===== %
    prob = 0;
    for i = 1:3
        % Check if the possible next state == next state
        % If so save the probability attached to that state
        if state_p(1:3,i) == state_n 
            prob = state_p(4,i);
        break;
    end     
end
