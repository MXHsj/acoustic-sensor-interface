%% read serial input
clc; clear; close all
port = 'COM3';

%% use serial method
% if ~isempty(sensors)
%     fclose(sensors);
%     delete(sensors);
% end
sensors = serial(port);

fopen(sensors); % initiate communication
% pause(2);
% while sensors.BytesAvailable > 0 %& grbl2.BytesAvailable > 0
%     echo = fscanf(sensors);
%     disp(echo);
% end
fscanf(sensors)
fclose(sensors);
delete(sensors);

%% 
if exist('sensors','var')
    clear sensors
end
sensors = serialport(port,9600,'DataBits',8,'Parity','none','StopBits',1);
% data = read(sensors,1,'double');
data = readbinblock(sensors);
% data = readline(sensors);

