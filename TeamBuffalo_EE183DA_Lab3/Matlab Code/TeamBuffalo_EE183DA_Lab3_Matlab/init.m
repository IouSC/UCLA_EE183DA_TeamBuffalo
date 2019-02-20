function [stateSpace,pe,lambda,s,action] = init()
% Initialization of value
value_int= zeros(6); 

% Expend the value matrix to 12 headings' 3D matrix space
[rows,cols] = size(value_int);
for x = 1:rows
    for y = 1:cols
        for h = 1:12
            stateSpace(x,y,h) = value_int(x,y);
        end
    end
end

% Initialization of error percentage (pe)
pe = 0.1;

% Initialization of discount factor
lambda = 0.9;

% Initialization of initial state
s = [2;5;6];

% Initialization of actions set
% column 1: forward 1 / no movement 0 / backward -1
% column 2: leftturn -1 / no rotation 0 / rightturn 1
action = [1  -1;...
          1  0;...
          1  1;...
          -1 -1;...
          -1 0;...
          -1 1;...
          0 0];
end