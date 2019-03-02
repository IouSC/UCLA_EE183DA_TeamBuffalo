function [sensor] = findmiddle(x)
    [cor1, cor2, cor3, cor4] = corners(x);
    sensor(1) = (cor1(1)+cor2(1))/2;
    sensor(2) = (cor1(2)+cor2(2))/2;
    sensor(3) = (cor1(1)+cor4(1))/2;
    sensor(4) = (cor1(2)+cor4(2))/2;
    sensor = sensor';  
end