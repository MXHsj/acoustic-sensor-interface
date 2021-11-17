%% read serial input
clc; clear; close all

port = 'COM3';
sensors = serialport(port,9600,'DataBits',8,'Parity','none','StopBits',1);

if ~isempty(sensors)
    fclose(sensors);
    delete(sensors);
end

fopen(sensors); % initiate grbl1 communication
pause(5);
while sensors.BytesAvailable > 0 %& grbl2.BytesAvailable > 0
    echo = fscanf(sensors);
    disp(echo);
end

fclose(sensors);
delete(ssensors);

% data = read(sensors,10,"double");