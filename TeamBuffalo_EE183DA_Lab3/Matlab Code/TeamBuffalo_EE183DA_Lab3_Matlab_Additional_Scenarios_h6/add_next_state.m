function [state_n,prerotation] = next_state(pe,state_c,action)
    % ===== pe && prerotation ===== %
    if (pe == 0)
        prerotation = 0;
    else
        % initialization of pe array
        pe_power = 0;
        pe_m = [];
        % calculate array size needed to account for pe's # of digits after decimal point
        while (floor(pe*10^pe_power) ~= pe*10^pe_power)
           pe_power = pe_power + 1; 
        end
        % create an array to store movements based on pe
        for i = 1:pe*power(10,pe_power)
            pe_m = [pe_m 1 -1];
        end
        pe_m = [pe_m zeros(1,power(10,pe_power) - 2*pe*power(10,pe_power))];
        % prerotation based on pe (defined in init.m)
        prerotation = pe_m(randi(power(10,pe_power)));
    end
    
    % ===== Initialize next state ===== %
    state_n = state_c;
    
    % ===== Update Next State ===== %
    % circular loop to account for the flaw in mod 12
    heading = [12 ,1 ,2, 3, 4, 5, 6, 7, 8, 9, 10, 11 ,12, 1];
    % Heading (prerotation)
    % account for pe = 0 (no error percentage), will not incur an error rotation
    % account for choosing to stay still, will not incur an error rotation
    if (pe ~= 0 && action(1) ~=0)
        state_n(3) = heading((state_n(3)+1) + prerotation);
    end
    % (x,y)
    % If statements account for when robot is at the border
    % when @ the border, robot can still rotate but cannot perform movement
    switch state_n(3)
        case {11,12,1}
            state_n(2) = state_c(2) + action(1);
            if (state_n(2) == 7 || state_n(2) == 0)
                state_n(2) = state_c(2);
            end 
        case {2,3,4}
            state_n(1) = state_c(1) + action(1);
            if (state_n(1) == 7 || state_n(1) == 0)
                state_n(1) = state_c(1);
            end 
        case {5,6,7}
            state_n(2) = state_c(2) - action(1);
            if (state_n(2) == 0 || state_n(2) == 7)
                state_n(2) = state_c(2);
            end 
        case {8,9,10}
            state_n(1) = state_c(1) - action(1); 
            if (state_n(1) == 0 || state_n(1) == 7)
                state_n(1) = state_c(1);
            end 
    end 
    % Heading 
    state_n(3) = heading((state_n(3)+1) + action(2));    
end
