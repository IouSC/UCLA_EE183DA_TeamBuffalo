function [cor1 cor2 cor3 cor4] = corners(x)
% Given current state, find the x,y coordinates of the 
% four corners of the car.
% Start with the sensors corner, rotate anticlockwise 1, 2, 3 ,4
    % Constant
    W = 100; %100mm
    L = 97; %97mm
    % Unpack
    rx = x(1);
    ry = x(2);
    theta =(x(3));  
    s = sind(theta);
    c = cosd(theta);
    cor1 = [rx, ry];
    cor2 = cor1 + [W*s, W*c];
    cor4 = cor1 + [-L*c, L*s];
    cor3 = cor4 + cor2 - cor1; 
end