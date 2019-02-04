%% EE 183DA
% Team Buffalo
% Lab 2 - Kalman Filter state estimator
clc; clear all; close all;

%% Load and subtract data
exp1 = load ('measure.txt');
exp2 = load ('measure_NEW.txt');
SENSOR = exp2(1:3,:);
LIDAR_F = exp2(1,:);
LIDAR_R = exp2(2,:);
MPU = exp2(3,:);
MODE = exp2(4,:);

%% Initial State
% exp1
%p_x = 150; 
%p_y = 100;
%p_theta = 333;
% exp2
p_x = 50; 
p_y = 200;
p_theta = 180;

x = [p_x; p_y; p_theta];

%% Sensor offset
offset = [28.236;
          63.228;
          0]; 
PtoL = [1.0065  0       0;
        0       1.0547  0;
        0       0       1];

%% Kalman Filter
% Variables and constant
I = eye(3);
A = I;
R = 0.1* eye(3);    % Measurement Noise
% Measurement Noise (Test in process)
%v = [];
%for i = 1:1000
%    v = [v, wgn(3,1,1)]; 
%end
%R = 0.01*v*v';

for i = 1:length(exp2)
    %estimation update
    x(:,i+1) = estimate(x(:,i),MODE(i));
    
    % z = Hx + V
    %mid = findmiddle(x(:,i))';
    [H,D] = linearize(x(:,i));
    %mid = findmiddle(x(:,i+1))'; 
    z(:,i) = PtoL * (H*x(:,i+1) + D) + offset;
    
    % (Test in process)
    %e = x(:,i) - x(:,i+1);
    %p = 0.01*e*e';
    p = 0.0001 * eye(3); 
    
    % measurement unpade
    K = p*H.'*inv(H*p*H.'+R); 
    x(:,i+1) = x(:,i+1) + K*(SENSOR(i)- z(:,i));
    p = (I - K*H) * p; 
end

%% Plot the car state
plot_car_state(x); 