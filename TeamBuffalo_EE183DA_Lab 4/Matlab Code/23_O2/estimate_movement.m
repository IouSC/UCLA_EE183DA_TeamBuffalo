function [y] = estimate_movement(x,command)   
    A = eye(3); 
    C_c = 56;
    theta = x(3);
    way = command(1); 
    t = command(2)/1000; 
    
    vb  = 0.101; %0.134 
    vf  = 0.117; %0.134
    vl  = 0.133; %0.305
    vr  = 0.118; %0.349
    
    switch way
        case 1
            %forward
            PWM_R = 63;
            PWM_L = 107;
            input = [PWM_R^2;...
                     PWM_R;... 
                     1;...
                     PWM_L^2;...
                     PWM_L;...
                     1];
            c = [-0.1863  19.934  -382.02 0       0       0;... 
                   0        0       0       0.016    -0.649  20.575]; 
            %v = mean(c*input);
            v = 117; 
            u = [v;...
                 v;...
                 theta];
            B = [t*cosd(theta)    0               0;...
                 0                -t*sind(theta)  0;...
                 0                0               0];
            y = A*x + B*u;
            if y(3) < 0
                y(3)= y(3) + 360;
            end 
            if y(3) > 360
                y(3)= y(3) - 360;
            end 
        case -1
            %backward
            PWM_R = 104;
            PWM_L = 65;
            input = [PWM_R^2;...
                     PWM_R;... 
                     1;...
                     PWM_L^2;...
                     PWM_L;...
                     1];
            c = [0.2874 -64.469 3462.5  0       0       0;...
                 0      0       0       0.2789  -31.524 735.09];
            
            %v = mean(c*input);
            v = -117; 
            u = [v;...
                 v;...
                 theta];
            B = [-t*cosd(theta)   0               0;...
                 0                t*sind(theta)   0;...
                 0                0               0];
            y = A*x - B*u;
            if y(3) < 0
                y(3)= y(3) + 360;
            end 
            if y(3) > 360
                y(3)= y(3) - 360;
            end
        case -2
            %left
            PWM_R = 64;
            PWM_L = 65;
            input = [PWM_R^2;... 
                     PWM_R;...
                     1;...
                     PWM_L^2;...
                     PWM_L;...
                     1];
            coeff = [-0.1062, 9.9628, -51.161, -0.0245, -2.6331, 428.17];
            %w = -1* mean(coeff*input);
            w  = -133;
            B = [C_c*(sind(theta) - sind(theta+w*t));... 
                 C_c*(cosd(theta) - cosd(theta+w*t));...
                 w*t];
            y = A*x + B;
            if y(3) < 0
                y(3)= y(3) + 360;
            end 
            if y(3) > 360
                y(3)= y(3) - 360;
            end 
        case 2
            %right
            PWM_R = 114;
            PWM_L = 105;
            input = [PWM_R^2;... 
                     PWM_R;...
                     1;...
                     PWM_L^2;...
                     PWM_L;...
                     1];
            coeff = [-0.0955, 23.74, -1287.7, -0.6161, 134.99, -7210.5];
            %w = 1*mean(coeff*input);
            w  = 118; 
            B = [C_c*(sind(theta) - sind(theta+w*t));... 
                 C_c*(cosd(theta) - cosd(theta+w*t));...
                 w*t];
            y = A*x + B;
            if y(3) < 0
                y(3)= y(3) + 360;
            end
            if y(3) > 360
                y(3)= y(3) - 360;
            end
        otherwise 
                y = x;
    end    
end