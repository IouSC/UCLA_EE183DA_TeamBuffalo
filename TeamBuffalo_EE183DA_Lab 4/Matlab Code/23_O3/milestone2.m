function [out,robot_path_dis] = milestone2(x,robot_path_dis)
    s = tcpip('192.168.4.1',80,'NetworkRole','client');
    fopen(s);
    R = 0.01* eye(3); 
    garbage = reading_sensor(s);
    reading = reading_sensor(s); 
    difference = reading(3) - 180; 
    kr = 6.5; 
    kl = 8.5; 
    if difference > 0
        send_command([2 (180-abs(difference))*kr],s); 
        x(:,2) = estimate_movement(x(:,1),[2 (180-abs(difference))*kr]);   
    else  
        send_command([-2 (180-abs(difference))*kl],s); 
        x(:,2) = estimate_movement(x(:,1),[-2 (180-abs(difference))*kl]);   
    end
    
    garbage = reading_sensor(s);
    reading = reading_sensor(s);
    
    % z = Hx + V
    [H,D,out] = linearize(x(:,2)); 
    
    p = 0.002*eye(3); 

    % measurement unpade
    k = p*H.'*inv(H*p*H.'+R); 
        different = reading-out; 
    if different(3)> 180 
        different(3) = different(3)-360; 
    elseif different(3) < -180 
        different(3) = 360-different(3);
    else
        different = different;
    end 
    
    x(:,2) =x(:,2) + k*(different);
    
    garbage = reading_sensor(s);
    reading = reading_sensor(s); 
    kb = 6; 
    difference = reading(1) - 30; 
    if difference >0
        send_command([1 difference*kb],s); 
        x(:,3) = estimate_movement(x(:,2),[1 difference]);   
    end
    
    
    flushinput(s); 
    garbage = reading_sensor(s);
    reading = reading_sensor(s);
    
    % z = Hx + V
    [H,D,out] = linearize(x(:,3)); 
    
    p = 0.002*eye(3); 

    % measurement unpade
    k = p*H.'*inv(H*p*H.'+R); 
        different = reading-out; 
    if different(3)> 180 
        different(3) = different(3)-360; 
    elseif different(3) < -180 
        different(3) = 360-different(3);
    else
        different = different;
    end 
    
    x(:,3) =x(:,3) + k*(different);
    
    robot_path_dis = plot_car(x,robot_path_dis); 
    out = x(:,3); 
    fclose(s);
end 