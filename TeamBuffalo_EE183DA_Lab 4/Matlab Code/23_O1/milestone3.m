function milestone3(x,robot_path_dis)
    s = tcpip('192.168.4.1',80,'NetworkRole','client');
    fopen(s);
    R = 0.01* eye(3); 
    garbage = reading_sensor(s);
    reading = reading_sensor(s); 
    difference = reading(3) - 90; 
    kr = 6.9; 
    kl = 8.8;
    x(:,2) = x(:,1); 
    if difference > 0
        send_command([-2 (reading(3)-270)*kl],s);
        x(:,3)= estimate_movement(x(:,2),[-2 0.9*(reading(3)-270)*kl]);   
    else  
        send_command([2 (90-reading(3))*kr],s);
        x(:,3) = estimate_movement(x(:,2),[2 (90-reading(3))*kr]);  
    end
       
    flushinput(s); 
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
    
    x(:,3) =x(:,3) + k*(different);
    plot_car(x,robot_path_dis); 
    fclose(s);
end 