function [cor1 cor2 cor3 cor4] = corners(x)
    % Car Constant
    W = 100;    % width of the car [mm]
    L = 97;     % length of the car [mm]
    % Find car reference point
    rx = x(1);
    ry = x(2);
    % Find the other three corners of the car
    theta =(x(3));  
    s = sind(theta);
    c = cosd(theta);
    cor1 = [rx, ry];
    cor2 = cor1 + [W*s, W*c];
    cor4 = cor1 + [-L*c, L*s];
    cor3 = cor4 + cor2 - cor1; 
end