%% read ch101 sensor values from serial input
clc; clear; close all

%% ----------------- ROS network -----------------
rosshutdown
% [~, local_ip] = system('ipconfig');
setenv('ROS_MASTER_URI','http://130.215.121.229:11311') % ip of robot desktop
setenv('ROS_IP','130.215.192.178')   % ip of this machine
rosinit
% define publisher
ch101_pub = rospublisher('ch101', 'std_msgs/Float64MultiArray');
ch101_msg = rosmessage(ch101_pub);
ch101_msg.Data = [nan, nan, nan, nan];

%% 
port = 'COM3';
n_sensors = 4;
if exist('sensors','var')
    clear sensors
end
sensors = serialport(port,9600,'DataBits',8,'Parity','none','StopBits',1);
dist = zeros(1,n_sensors);
port_flag = zeros(1,n_sensors,'logical');

while 1
    tic
    latest = char(readline(sensors));
    % check which port
    for i=1:n_sensors
        port_flag(i) = contains(latest,[num2str(i-1),':']);
    end
    % extract distance
    range_ind = strfind(latest, 'Range: ');
    if ~isempty(range_ind)
        dist(port_flag) = str2double(latest(range_ind+7:range_ind+11));
    else
        dist(port_flag) = nan;
    end
    % vis
    fprintf('port0 range: [mm]: %f \n', dist(1))
    fprintf('port1 range: [mm]: %f \n', dist(2))
    fprintf('port2 range: [mm]: %f \n', dist(3))
    fprintf('port3 range: [mm]: %f \n', dist(4))
    
    ch101_msg.data = dist;
    send(ch101_pub, ch101_msg);
    toc;
end