function [] = send_command(input,s)
    process_time = input(2);
    pro_delay = 2; 
    command = input(1);

    switch command
        case 1
            action = 70; 
        case -1
            action = 66;
        case -2
            action = 76;
        case 2
            action = 82; 
        otherwise 
            action = 83;
    end 

    time_floor = floor(process_time/256);
    time_mod = mod(process_time,256);
    command_w_time = uint8([time_floor,time_mod,action]);
    fwrite(s,command_w_time);
    pause(pro_delay+process_time/1000); 
end