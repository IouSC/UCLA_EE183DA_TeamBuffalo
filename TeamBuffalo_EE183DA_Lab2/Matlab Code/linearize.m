function [H, D] = linearize(x)
    % Symbols and Constants
    L = 762.5;  % Length of the box [mm] 
    W = 495;    % Width of the box [mm]
    Lf_c = 50;  % front lidar sensor to the car sensor reference point [mm]
    Lr_c = 53;  % right lidar sensor to the car sensor reference point [mm]
    
    % MPU9250
    p_theta = x(3);
    % Lidar
    p_x = x(1);
    p_y = x(2);
    
    if p_theta < 0
        p_theta = p_theta + 360;
    end
    if p_theta > 360
        p_theta = p_theta - 360;
    end

    % Cases
    switch p_theta
    case 0
        H = [-1 0   0; 
             0  1   0; 
             0  0   1];
        D = [L;
             0;
             0];
    case 360
        H = [-1 0   0;
             0  1   0;
             0  0   1];
        D = [L;
             0;
             0];
    case 180
        H = [1  0   0;
             0  -1  0;
             0  0   1]; 
        D = [0;
             W;
             0]; 
    case 90
        H = [0  1   0;
             1  0   0;
             0  0   1];
        D = [0;
             0;
             0];
    case 270
        H = [0  -1  0;
             -1 0   0;
             0  0   1];
        D = [0;
             0;
             0];
    otherwise
        if p_theta < 0
            p_theta = p_theta + 360; 
        end
        if p_theta < 90
            xf = L;
            yf = 0;
            xr = 0;
            yr = 0; 
        elseif p_theta <180 && p_theta>90
            xf = 0;
            yf = 0;
            xr = 0;
            yr = W;
        elseif p_theta <270 && p_theta>180
            xf = 0;
            yf = W;
            xr = L;
            yr = W;
        elseif p_theta <360 && p_theta>270
            xf = L;
            yf = W;
            xr = L;
            yr = 0;
        end
        
        % Lidar (Test in process)
        %p_y = x(2) + Lf_c*cosd(p_theta);
        %f_p_x = x(1) + Lf_c*sind(p_theta);
        %r_p_x = x(1) - Lf_c*sind(p_theta);
        %fx = f_p_x + (p_y - yf) / tan(p_theta); 
        %fy = p_y + (f_p_x - xf) * tan(p_theta); 
        %rx = r_p_x + (yr - p_y) * tan(p_theta); 
        %ry = p_y + (xr - r_p_x) / tan(p_theta);
        
        fx = p_x + (p_y - yf) / tan(p_theta); 
        fy = p_y + (p_x - xf) * tan(p_theta); 
        rx = p_x + (yr - p_y) * tan(p_theta); 
        ry = p_y + (xr - p_x) / tan(p_theta);
        
        points = [xf fy;
                  fx yf;
                  rx yr;
                  xr ry];
        
        for i = 1:2
            % (Test in process)
%             if i == 1
%                 p_x = f_p_x;
%             else if i == 2
%                 p_x = r_p_x;
%             end
            tempA = pdist([p_x, p_y; points(2*i-1,:)]); 
            tempB = pdist([p_x, p_y; points(2*i,:)]); 
            if tempA < tempB
                p = [points(2*i-1,:) tempA];
            else
                p = [points(2*i,:) tempB];
            end 
            H(i,:) = [(p_x - p(1))/p(3),(p_y - p(2))/p(3),0];
            D(i,:) = [(-p_x + p(1))/p(3)*p_x - (p_y - p(2))/p(3)*p_y + p(3)];
        end 
        H(3,:) = [0 0 1];
        D(3) = 0;  
    end
end
