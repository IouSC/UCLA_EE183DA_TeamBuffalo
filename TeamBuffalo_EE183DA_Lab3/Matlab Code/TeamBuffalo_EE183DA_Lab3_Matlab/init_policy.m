function [policy] = init_policy(rows)
    % ===== Initialization of initial policy \pi_0 ===== %
    % REFER to lab 3 instructions for the initialization process
        % If the goal is in front of you, move forward
        % If it is behind you, move backward
        % Then turn the amount that aligns your next direction of travel closer towards the goal (if necessary).
        % If the goal is directly to your left or right, move forward then turn appropriately.
    for x = 1:6
        for y = 1:6
            % Iterate through all headings
            for h = 1:12
                % Case 1: (x,y) = (<5,<5)
                if (x < 5 && (rows+1) - y < 5)
                    switch h
                        case {11,12,1}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = 1;
                        case {2,3,4}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = -1; 
                        case {5,6,7}
                             policy(y,x,h,1) = -1;
                             policy(y,x,h,2) = -1; 
                        case {8,9,10}
                             policy(y,x,h,1) = -1;
                             policy(y,x,h,2) = 1;  
                    end             
                end
                % Case 2: (x,y) = (<5,5)
                if (x < 5 && (rows+1) - y == 5)
                    switch h
                        case {11,12,1}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = 1;
                        case {2,3,4}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = 0; 
                        case {5,6,7}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = -1; 
                        case {8,9,10}
                             policy(y,x,h,1) = -1;
                             policy(y,x,h,2) = 0;  
                    end             
                end
                % Case 3: (x,y) = (<5,6)
                if (x < 5 && (rows+1) - y == 6)
                    switch h
                        case {11,12,1}
                             policy(y,x,h,1) = -1;
                             policy(y,x,h,2) = 1;
                        case {2,3,4}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = 1; 
                        case {5,6,7}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = -1; 
                        case {8,9,10}
                             policy(y,x,h,1) = -1;
                             policy(y,x,h,2) = -1; 
                    end             
                end
                % Case 4: (x,y) = (5,<5)
                if (x == 5 && (rows+1) - y < 5)
                    switch h
                        case {11,12,1}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = 0;
                        case {2,3,4}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = -1; 
                        case {5,6,7}
                             policy(y,x,h,1) = -1;
                             policy(y,x,h,2) = 0; 
                        case {8,9,10}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = 1; 
                    end
                end
                % Case 5: (x,y) = (5,5) REACHED GOAL SQUARE
                if (x == 5 && (rows+1) - y == 5)
                   policy(y,x,h,1) = 0;
                   policy(y,x,h,2) = 0;
                end
                % Case 6: (x,y) = (5,6)
                if (x == 5 && (rows+1) - y == 6)
                    switch h
                        case {11,12,1}
                             policy(y,x,h,1) = -1;
                             policy(y,x,h,2) = 0;
                        case {2,3,4}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = 1; 
                        case {5,6,7}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = 0; 
                        case {8,9,10}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = -1; 
                    end             
                end
                % Case 7: (x,y) = (6,<5)
                if (x == 6 && (rows+1) - y < 5)
                    switch h
                        case {11,12,1}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = -1;
                        case {2,3,4}
                             policy(y,x,h,1) = -1;
                             policy(y,x,h,2) = -1; 
                        case {5,6,7}
                             policy(y,x,h,1) = -1;
                             policy(y,x,h,2) = 1; 
                        case {8,9,10}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = 1;  
                    end             
                end
                % Case 8: (x,y) = (6,5)
                if (x == 6 && (rows+1) - y == 5)
                    switch h
                        case {11,12,1}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = -1;
                        case {2,3,4}
                             policy(y,x,h,1) = -1;
                             policy(y,x,h,2) = 0; 
                        case {5,6,7}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = 1; 
                        case {8,9,10}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = 0;  
                    end             
                end
                % Case 9: (x,y) = (6,6)
                if (x == 6 && (rows+1) - y ==6)
                    switch h
                        case {11,12,1}
                             policy(y,x,h,1) = -1;
                             policy(y,x,h,2) = -1;
                        case {2,3,4}
                             policy(y,x,h,1) = -1;
                             policy(y,x,h,2) = 1; 
                        case {5,6,7}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = 1; 
                        case {8,9,10}
                             policy(y,x,h,1) = 1;
                             policy(y,x,h,2) = -1;  
                    end             
                end
            end    
        end
    end
end
