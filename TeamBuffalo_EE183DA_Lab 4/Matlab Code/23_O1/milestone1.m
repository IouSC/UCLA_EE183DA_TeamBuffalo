function [out,robot_path_dis]= milestone1(pos_init,s)
    R = 0.01* eye(3); 
    x = [pos_init(1); pos_init(2); pos_init(3)]; 
    A = eye(3); 
    s = tcpip('192.168.4.1',80,'NetworkRole','client');
    fopen(s);

    % Calculate total distance traveled by the robot;
    robot_path_dis = 0;
    annotationStr = ['Distance of the path robot taken = ' num2str(robot_path_dis) ' [mm]'];
    annotation('textbox',[.1 .875 .5 .05],'String',annotationStr,'interpreter','latex',...
               'FitBoxToText','on','EdgeColor','none');

    while 1
        % estimation update 
        action =  get_command(x(:,1)); 
        if action(1,1) ==0 && action(1,2) ==0
            break
        end 
        x(:,2) = estimate_movement(x(:,1),action(:,2));
        send_command(action(:,2),s); 

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

        x(:,2) =x(:,2) + k*(different);
        %x(3) = reading(3); 
        p = (eye(3) - k*H) * p;

        x(:,3) = estimate_movement(x(:,2),action(:,1));
        send_command(action(:,1),s);

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

        x(:,3) = x(:,3) + k*(different);
        p = (eye(3) - k*H) * p; 
        robot_path_dis = plot_car(x,robot_path_dis);
        x(:,1) = x(:,3);

        out = x(:,3); 
    end
    
    fclose(s);
end