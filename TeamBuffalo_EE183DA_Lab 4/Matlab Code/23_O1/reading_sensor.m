function [reading] = reading_sensor(s)

pro_delay = 2; 

%X = 88
front_sensor = uint8([0,0,88]);
fwrite(s,front_sensor);

%Y = 89
side_sensor = uint8([0,0,89]);
fwrite(s,side_sensor);

%H = 72
heading = uint8([0,0,72]);
fwrite(s,heading);

sensor_reading = fread(s,3,'uint16');

reading = sensor_reading;
pause(pro_delay); 
end