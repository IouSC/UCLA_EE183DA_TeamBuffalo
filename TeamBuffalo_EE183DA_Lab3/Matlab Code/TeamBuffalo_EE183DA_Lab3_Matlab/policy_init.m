%% EE 183DA Lab 3
% Team Buffalo
    % Iou-Sheng (Danny) Chang   UID: 804-743-003
    % William           Argus   UID: 004-610-455
    % XianXing  (Gray)  Jiang   UID: 604-958-018
    % Ho        (Bobby) Dong    UID: 604-954-176
% Markov Decision Processes (MDPs)
clc; clear all; close all;

%% For initial policy \pi_0
robot = zeros(6);
[rows,cols] = size(robot);

% Initialization
[optimal_value_policy,pe,lambda,s,action] = init();
policy_p = init_policy(rows); 
prer = 0;
pe = 0;

val = get_value(policy_p,lambda,pe,rows);

% Robot trajectory plot using initial policy
% NOTE: comment out rest of the code in section 2.3 before running this part
 % Initialization

robot((rows+1)-s(2),s(1)) = s(3);
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
% 2.3(c)
fprintf('========== 2.3(c) ==========\n');
fprintf('REFER to figure plotted or video linked in the lab report reference\n')

% 2.3(e)
fprintf('========== 2.3(e) ==========\n');
fprintf('REFER to ''val'' matrix\n');
