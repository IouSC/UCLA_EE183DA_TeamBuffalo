function [y] = estimate(x,mode)
    % variables and constants
    A = eye(3); 
    C_c = 56;
    theta = x(3);
    % Four modes of moving
    switch mode
        % Forward mode
        case 1
            PWM_R = 63;
            PWM_L = 107;
            t = 0.065;
            input = [PWM_R^2;
                     PWM_R; 
                     1;
                     PWM_L^2;
                     PWM_L;
                     1];
            c = [-0.1863  19.934  -382.02   0       0       0;
                 0        0       0         0.016   -0.649  20.575]; 
            v = mean(c*input);
            u = [v;
                 v;
                 theta];
            B = [t*cosd(theta)    0               0;
                 0                -t*sind(theta)  0;
                 0                0               0];
            y = A*x + B*u;
        % Backward mode
        case 2
            PWM_R = 104;
            PWM_L = 65;
            t = 0.065;
            input = [PWM_R^2;
                     PWM_R; 
                     1;
                     PWM_L^2;
                     PWM_L;
                     1];
            c = [0.2874 -64.469 3462.5  0       0       0;
                 0      0       0       0.2789  -31.524 735.09];
            v = mean(c*input);
            u = [-v;
                 -v;
                 theta];
            B = [-t*cosd(theta)   0               0;
                 0                t*sind(theta)   0;
                 0                0               0];
            y = A*x + B*u;
        % Left turn mode
        case 3
            PWM_R = 64;
            PWM_L = 65;
            t = 0.045;
            input = [PWM_R^2; 
                     PWM_R;
                     1;
                     PWM_L^2;
                     PWM_L;
                     1];
            coeff = [-0.1062, 9.9628, -51.161, -0.0245, -2.6331, 428.17];
            w = -1* mean(coeff*input);
            C_c= C_c*0.3; 
            B = [-C_c*(cosd(theta) - cosd(theta + w*t));
                 C_c*(sind(theta) - sind(theta + w*t)); 
                 w*t];
            y = A*x + B;
        % Right turn mode
        case 4
            PWM_R = 114;
            PWM_L = 105;
            t = 0.03;
            input = [PWM_R^2; 
                     PWM_R;
                     1;
                     PWM_L^2;
                     PWM_L;
                     1];
            coeff = [-0.0955, 23.74, -1287.7, -0.6161, 134.99, -7210.5];
            w = 1*mean(coeff*input);
            B = [C_c*(sind(theta) - sind(theta + w*t));
                 C_c*(cosd(theta) - cosd(theta + w*t));
                 w*t]; 
            %w = 1*mean(coeff*input);
            %B = [-C_c*(cosd(theta) - cosd(theta + w*t));
            %     C_c*(sind(theta) - sind(theta + w*t));
            %     w*t]; 
            y = A*x + B;
        otherwise 
            y = x;
    end
    
    if y(3) < 0
        y(3) = y(3) + 360;
    end
    if y(3) > 360
        y(3) = y(3) - 360;
    end 
end