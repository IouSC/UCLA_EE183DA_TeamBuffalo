function [mid_f mid_r mid] = findmiddle(x)
    [cor1, cor2, cor3, cor4] = corners(x);
    mid_f = (cor1 + cor2)/2;
    mid_r = (cor1 + cor4)/2;
    mid = (cor1 + cor3)/2; 
end