%% 2.5 Policy Iteration
% Robot State Initialization
clc; clear all; close all;

robot = zeros(6);
[rows,cols] = size(robot);


% Initialization
[value,reward,pe,lambda,s,action] = init();
policy_a = init_policy(rows);  
prer = 0;

% Using MATLAB built in 'tic toc' command to obtain compute time
tic
[policy_a,value] = policy(policy_a,lambda,pe,rows,action,reward); 
computeTime_policyIterationStr = toc; 

% Display initial state
robot((rows+1)-s(2),s(1)) = s(3);

% Plot robot state
% Grid World has the same configuration as the figure shown in lab3 instruction
% Value shown on the graph is the heading
% Next action and prerotation is shown in the x axis
% when we reach the goal block, break out of the loop 
for j = 1:1000
    act = policy_a((rows+1)-s(2),s(1),s(3),:);
    plot_route(robot,prer,act);
    optimal_policy_policy(j,1) = act(1,1,1,1);
    optimal_policy_policy(j,2) = act(1,1,1,2);
    if (s(1) == 5 && s(2) == 5 && s(3) == 6)
        break; 
    end
    robot((rows+1)-s(2),s(1)) = 0; 
    [s prer]= next_state(pe,s,act);
    robot((rows+1)-s(2),s(1)) = s(3);
end 

% Command Window
% 2.5(b)
fprintf('========== 2.5(b) ==========\n');
fprintf('REFER to figure plotted or video linked in the lab report reference\n');
computeTime_policyIterationStr = ['Policy Iteration Compute Time = ',num2str(computeTime_policyIterationStr),' [s]'];
fprintf(computeTime_policyIterationStr);
