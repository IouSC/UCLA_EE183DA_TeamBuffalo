function[command] = get_command(x)
x_1 = x(1); 
y_1 = x(2); 
theta_1 = x(3);

[cor1, cor2, ~, ~] = corners(x);
mid(1) = (cor1(1)+cor2(1))/2
mid(2) = (cor1(2)+cor2(2))/2

if abs(mid(1)-525)<15 && abs(mid(2)-200)<15
    command = [0 0; 0 0]; 
    return
end 

[x_2,y_2] = RRT_star(mid(1),mid(2)); 
theta_2 = atan2(y_2-mid(2),x_2-mid(1))/pi*180;

vb  = 0.101; %0.134 
vf  = 0.117; %0.134
vl  = 0.133; %0.305
vr  = 0.118; %0.349

left = -2; 
right = 2;
forward = 1; 
backward = -1; 

d = pdist([x_2 y_2; mid(1) mid(2)]); 

if  theta_2 >= 0 && theta_2 < 180
    theta_2 = 360 - theta_2;
else 
    theta_2 = -1*theta_2;
end 

if theta_2 >270 && theta_2 <= 360
    if  theta_1 >= theta_2
        %left and forward
        command(1,:) = [d theta_1-theta_2];
        command(2,:) = [command(1,1)/vf command(1,2)/vl];
        command(1,:) = [forward left];
    elseif theta_1 > theta_2-90 && theta_1 <= theta_2
        %right and forward
        command(1,:) = [d theta_2-theta_1]; 
        command(2,:) = [command(1,1)/vf command(1,2)/vr];
        command(1,:) = [forward right];
    elseif theta_1 > theta_2-180 && theta_1 <= theta_2-90
        %left and backward
        command(1,:) = [d theta_1+180-theta_2];
        command(2,:) = [command(1,1)/vb command(1,2)/vl];
        command(1,:) = [backward left];
    elseif theta_1 > theta_2-270 && theta_1 <= theta_2-180
        %right and backward
        command(1,:) = [d theta_2-180-theta_1];
        command(2,:) = [command(1,1)/vb command(1,2)/vr];
        command(1,:) = [backward right];
    elseif theta_1 >= 0 && theta_1 <= theta_2-270
        %left and forward
        command(1,:) = [d 360-theta_2+theta_1];
        command(2,:) = [command(1,1)/vf command(1,2)/vl];
        command(1,:) = [forward left];
    end
elseif theta_2 > 180 && theta_2 <= 270
    if  theta_1 > theta_2 && theta_1 <= theta_2+90
        %right and forward
        command(1,:) = [d theta_1-theta_2];
        command(2,:) = [command(1,1)/vf command(1,2)/vl];
        command(1,:) = [forward left];
    elseif theta_1 > theta_2-90 && theta_1 <= theta_2
        %right and forward
        command(1,:) = [d theta_2-theta_1]; 
        command(2,:) = [command(1,1)/vf command(1,2)/vr];
        command(1,:) = [forward right];
    elseif theta_1 > theta_2-180 && theta_1 <= theta_2-90
        %left and backward
        command(1,:) = [d theta_1-theta_2+180];
        command(2,:) = [command(1,1)/vb command(1,2)/vl];
        command(1,:) = [backward left];
    elseif theta_1 >= 0 && theta_1 <= theta_2-180
        %right and backward
        command(1,:) = [d theta_2-180-theta_1];
        command(2,:) = [command(1,1)/vf command(1,2)/vr];
        command(1,:) = [backward right];
    elseif theta_1 >= theta_2+90
        %right and backward
        command(1,:) = [d theta_2+180-theta_1];
        command(2,:) = [command(1,1)/vb command(1,2)/vr];
        command(1,:) = [backward right];
    end
elseif theta_2 > 90 && theta_2 <= 180
   if  theta_1 > theta_2 && theta_1 <= theta_2+90
        %left and forward
        command(1,:) = [d theta_1-theta_2];
        command(2,:) = [command(1,1)/vf command(1,2)/vl];
        command(1,:) = [forward left];
    elseif theta_1 > theta_2-90 && theta_1 <= theta_2
        %right and forward
        command(1,:) = [d theta_2-theta_1]; 
        command(2,:) = [command(1,1)/vf command(1,2)/vr];
        command(1,:) = [forward right];
    elseif theta_1 >= 0 && theta_1 <= theta_2-90
        %left and backward
        command(1,:) = [d theta_1+180-theta_2];
        command(2,:) = [command(1,1)/vb command(1,2)/vl];
        command(1,:) = [backward left];
    elseif theta_1 > theta_2+180 
        %left and backward
        command(1,:) = [d theta_1-180-theta_2];
        command(2,:) = [command(1,1)/vb command(1,2)/vl];
        command(1,:) = [backward left];
    elseif theta_1 > theta_2+90 && theta_1 <= theta_2+180
        %right and backward
        command(1,:) = [d theta_2+180-theta_1];
        command(2,:) = [command(1,1)/vb command(1,2)/vr];
        command(1,:) = [backward right];
    end
elseif theta_2 >= 0 && theta_2 <= 90
    if  theta_1 > theta_2 && theta_1 <= theta_2+90
        %left and forward
        command(1,:) = [d theta_1-theta_2];
        command(2,:) = [command(1,1)/vf command(1,2)/vl];
        command(1,:) = [forward left];
    elseif theta_1 > theta_2+90 && theta_1 <= theta_2+180
        %right and backward
        command(1,:) = [d theta_2+180-theta_1]; 
        command(2,:) = [command(1,1)/vb command(1,2)/vr];
        command(1,:) = [backward right];
    elseif theta_1 > theta_2+180 && theta_1 <= theta_2+270
        %left and forward
        command(1,:) = [d theta_1-180-theta_2];
        command(2,:) = [command(1,1)/vf command(1,2)/vl];
        command(1,:) = [backward left];
    elseif theta_1 > theta_2+270 
        %right and forward
        command(1,:) = [d 360-theta_1+theta_2];
        command(2,:) = [command(1,1)/vf command(1,2)/vr];
        command(1,:) = [forward right];
    elseif theta_1 >= 0 && theta_1 < theta_2
        %right and forward
        command(1,:) = [d theta_2-theta_1];
        command(2,:) = [command(1,1)/vf command(1,2)/vr];
        command(1,:) = [forward right];
    end
end 
end 




