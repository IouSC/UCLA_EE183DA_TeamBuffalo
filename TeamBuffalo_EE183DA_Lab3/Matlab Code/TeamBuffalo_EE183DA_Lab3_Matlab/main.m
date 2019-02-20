%% EE 183DA Lab 3
% Team Buffalo
    % Iou-Sheng (Danny) Chang   UID: 804-743-003
    % William           Argus   UID: 004-610-455
    % XianXing  (Gray)  Jiang   UID: 604-958-018
    % Ho        (Bobby) Dong    UID: 604-954-176
% Markov Decision Processes (MDPs)
clc; clear all; close all;

%% Robot State Initialization
robot = zeros(6);
[rows,cols] = size(robot);

%% 2.1 MDP System
% 2.1(a)
% REFER to stateSpace R^{3} matrix
fprintf('========== 2.1(a) ==========\n');
fprintf('REFER to stateSpace R^{3} matrix in ''init.m''\n');
% 2.1(b)
% REFER to action R^{2} matrix
fprintf('========== 2.1(b) ==========\n');
fprintf('REFER to action R^{2} matrix in ''init.m''\n');
% 2.1(c)
% REFER to 'probability.m' function for detail
fprintf('========== 2.1(c) ==========\n');
fprintf('REFER to ''probability.m'' function for detail\n');

% 2.1(d)
% REFER to 'next_state.m' function for detail
fprintf('========== 2.1(d) ==========\n');
fprintf('REFER to ''next_state.m'' function for detail\n');

%% 2.2 Planning problem
% 2.2(a)
% REFER to 'reward.m' function for detail
% Use boolean to check given state if return correct reward value
fprintf('========== 2.2(a) ==========\n');
fprintf('REFER to ''reward.m'' function for detail\n');
% border states (reward = -100)
fprintf('border states reward value = -100 (yes(1) / no(0)): %d\n',...
        (reward([1,1]) == -100) && (reward([1,2]) == -100) && (reward([1,3]) == -100) && (reward([1,4]) == -100) && (reward([1,5]) == -100) &&...
        (reward([6,2]) == -100) && (reward([6,3]) == -100) && (reward([6,4]) == -100) && (reward([6,5]) == -100) && (reward([6,6]) == -100) &&...
        (reward([2,1]) == -100) && (reward([3,1]) == -100) && (reward([4,1]) == -100) && (reward([5,1]) == -100) && (reward([6,1]) == -100) &&...
        (reward([1,6]) == -100) && (reward([2,6]) == -100) && (reward([3,6]) == -100) && (reward([4,6]) == -100) && (reward([5,6]) == -100));
% lane markers (reward = -10)
fprintf('lane markers reward value = -10 (yes(1) / no(0)): %d\n',...
        (reward([4,4]) == -10) && (reward([4,5]) == -10));
% goal square (reward = 1)
fprintf('goal square reward value = 1 (yes(1) / no(0)): %d\n',...
        (reward([5,5]) == 1));
% other states (reward = 0)
fprintf('other states reward value = 0 (yes(1) / no(0)): %d\n',...
        (reward([2,2]) == 0) && (reward([2,3]) == 0) && (reward([2,4]) == 0) && (reward([2,5]) == 0) &&...
        (reward([3,2]) == 0) && (reward([3,3]) == 0) && (reward([3,4]) == 0) && (reward([3,5]) == 0) &&...
        (reward([4,2]) == 0) && (reward([4,3]) == 0) &&...
        (reward([5,2]) == 0) && (reward([5,3]) == 0) && (reward([5,4]) == 0));

%% 2.3 Policy Iteration
% Use 'Run Section' to only run the policy iteration
clc; clear all; close all;

% Robot State Initialization
robot = zeros(6);
[rows,cols] = size(robot);

% Initialization
[optimal_value_policy,pe,lambda,s,action] = init();
policy_p = init_policy(rows); 
prer = 0;

% Using MATLAB built in 'tic toc' command to obtain compute time
tic
[policy_p,optimal_value_policy] = policy(policy_p,lambda,pe,rows,action);  
computeTime_policyIterationStr = toc;

% Display initial state
robot((rows+1)-s(2),s(1)) = s(3);

% Plot robot state
% Grid World has the same configuration as the figure shown in lab3 instruction
% Value shown on the graph is the heading
% Next action and prerotation is shown in the x axis
% when we reach the goal block, break out of the loop 
for j = 1:1000
    act = policy_p((rows+1)-s(2),s(1),s(3),:);
    plot_route(robot,prer,act);
    optimal_policy_policy(j,1) = act(1,1,1,1);
    optimal_policy_policy(j,2) = act(1,1,1,2);
    if (s(1) == 5 && s(2) ==5)
        break; 
    end
    robot((rows+1)-s(2),s(1)) = 0; 
    [s prer]= next_state(pe,s,act);
    robot((rows+1)-s(2),s(1)) = s(3);
end 

% Command Window
% 2.3(a)
fprintf('========== 2.3(a) ==========\n');
fprintf('REFER to ''init_policy.m'' for detail\n');
% 2.3(b)
fprintf('========== 2.3(b) ==========\n');
fprintf('REFER to ''plot_route.m'' for detail\n');
% 2.3(c)
fprintf('========== 2.3(c) ==========\n');
fprintf('REFER to ''policy_init.m'' for detail\n');
fprintf('REFER to figure plotted or video linked in the lab report reference\n')
% 2.3(d)
fprintf('========== 2.3(d) ==========\n');
fprintf('REFER to ''get_value.m'' for detail\n');
% 2.3(e)
fprintf('========== 2.3(e) ==========\n');
fprintf('REFER to ''policy_init.m'' for detail\n');
% 2.3(f)
fprintf('========== 2.3(f) ==========\n');
fprintf('REFER to ''get_policy.m'' for detail\n');
% 2.3(g)
fprintf('========== 2.3(g) ==========\n');
fprintf('REFER to ''policy.m'' for detail\n');
% 2.3(h)
fprintf('========== 2.3(h) ==========\n');
fprintf('REFER to figure plotted or video linked in the lab report reference\n');
% 2.3(i)
fprintf('========== 2.3(i) ==========\n');
computeTime_policyIterationStr = ['Policy Iteration Compute Time = 4.6858 [s]'];
%computeTime_policyIterationStr = ['Policy Iteration Compute Time = ',num2str(computeTime_policyIterationStr),' [s]'];
fprintf(computeTime_policyIterationStr);
    
%% 2.4 Value iteration
% Use 'Run Section' to only run the value iteration
clc; clear all; close all;

% Robot State Initialization
robot = zeros(6);
[rows,cols] = size(robot);

% Initialization
[value_prev,pe,lambda,s,action] = init();
optimal_value_value = value_prev; 
value_difference = 10e5;
prer = 0;

% Using MATLAB built in 'tic toc' command to obtain compute time
tic
while value_difference > 10e-5
    [value_prev,policy_v] = value(optimal_value_value,action,lambda,pe,rows);
    value_difference = sum(sum(sum(abs(value_prev-optimal_value_value))));
    optimal_value_value = value_prev; 
end 
computeTime_valueIteration = toc;

% Display initial state
robot((rows+1)-s(2),s(1)) = s(3);

% Plot robot state
% Grid World has the same configuration as the figure shown in lab3 instruction
% Value shown on the graph is the heading
% Next action and prerotation is shown in the x axis
% when we reach the goal block, break out of the loop 
for j = 1:1000
    act = policy_v((rows+1)-s(2),s(1),s(3),:);
    plot_route(robot,prer,act);
    optimal_policy_value(j,1) = act(1,1,1,1);
    optimal_policy_value(j,2) = act(1,1,1,2);
    if (s(1) == 5 && s(2) ==5)
        break; 
    end
    robot((rows+1)-s(2),s(1)) = 0; 
    [s prer]= next_state(pe,s,act);
    robot((rows+1)-s(2),s(1)) = s(3);
end 

% Command Window
% 2.4(a)
fprintf('========== 2.4(a) ==========\n');
fprintf('REFER to ''optimal_policy_value'' matrix\n');
fprintf('REFER to ''optimal_value_value'' matrix\n');
% 2.4(b)
fprintf('========== 2.4(b) ==========\n');
fprintf('REFER to figure plotted or video linked in the lab report reference\n');
% 2.4(c)
fprintf('========== 2.4(c) ==========\n');
computeTime_valueIterationStr = ['Value Iteration Compute Time = ',num2str(computeTime_valueIteration),' [s]'];
fprintf(computeTime_valueIterationStr);