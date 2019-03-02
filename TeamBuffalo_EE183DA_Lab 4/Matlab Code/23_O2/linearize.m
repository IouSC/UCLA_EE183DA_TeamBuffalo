function [H, D, out] = linearize(x)
    % Symbols and Constants
    L = 762.5;    % Length of the box [mm] 
    W = 495;    % Width of the box [mm]
    % Case
    p_x = x(1);
    p_y = x(2);
    p_theta = x(3);
    if p_theta < 0
        p_theta =p_theta + 360;
    end 
    if p_theta > 360
        p_theta = p_theta - 360;
    end 
    switch p_theta
    case 0
        H = [-1 0 0; 0 1 0; 0 0 1];
        D = [L ;0 ;0];
        d = [(L-p_x)^2; (L-p_x); 1; p_y^2; p_y;1; p_theta];
    case 360
        H = [-1 0 0; 0 1 0; 0 0 1];
        D = [L ;0 ;0];
        d = [(L-p_x)^2; (L-p_x); 1; p_y^2; p_y;1; p_theta];
    case 180
        H = [1 0 0; 0 -1 0; 0 0 1];
        D = [0; W; 0]; 
        d = [(p_x)^2; p_x; 1; (W-p_y)^2; W-p_y;1; p_theta];
    case 90
        H = [0 -1 0; -1 0 0; 0 0 1];
        D = [W; L; 0];
        d = [(p_y)^2; p_y;1;(p_x)^2; p_x; 1; p_theta];
    case 270
        H = [0 1 0; 1 0 0; 0 0 1];
        D = [0; 0; 0];
        d = [(W-p_y)^2; W-p_y;1;(L-p_x)^2; L-p_x; 1; p_theta];
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
        
        for i = 1:2
            sensor = findmiddle(x); 
            p_x = sensor(2*i-1);
            p_y = sensor(2*i);
            fx = p_x + (p_y-yf)/tand(p_theta); 
            fy = p_y + (p_x-xf)*tand(p_theta); 
            rx = p_x + (yr-p_y)*tand(p_theta); 
            ry = p_y + (xr-p_x)/tand(p_theta);
            points = [xf fy;fx yf;rx yr;xr ry];
            tempA = pdist([p_x, p_y; points(2*i-1,:)]); 
            tempB = pdist([p_x, p_y; points(2*i,:)]); 
            if tempA < tempB
                p = [points(2*i-1,:) tempA];  
            else p = [points(2*i,:) tempB];  
            end 
            H(i,:) = [(-p(1) + p_x)/p(3), (-p(2) + p_y)/p(3) 0];
            D(i,:) = [(p(1) - p_x)/p(3)*p_x + (p(2) - p_y)/p(3)*p_y + p(3)];
            d(3*i-2,:) = p(3)^2; 
            d(3*i-1,:) =p(3);
            d(3*i,:) =1;
        end 
        H(3,:) = [0 0 1];
        d(7,:) = p_theta;
        D(3) = 0;
    end
    offset = [47.669; 49.51; 0]; 
    PtoL = [0 1.01126 0 0 0 0 0; 0 0 0 -0.0006 1.2564 0 0; 0 0 0 0 0 0 1];
    out = PtoL *d + offset;
    
end
