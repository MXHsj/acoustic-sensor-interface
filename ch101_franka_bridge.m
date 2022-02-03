%% read ch101 sensor values from serial input
clc; clear; close all

%% ----------------- ROS network -----------------
rosshutdown
% [~, local_ip] = system('ipconfig');
setenv('ROS_MASTER_URI','http://130.215.211.193:11311') % ip of robot desktop
setenv('ROS_IP','130.215.192.168')   % ip of this machine
rosinit
% define publisher
ch101_dist_pub = rospublisher('ch101/distance', 'std_msgs/Float64MultiArray');
ch101_norm_pub = rospublisher('ch101/normal', 'std_msgs/Float64MultiArray');
ch101_dist_msg = rosmessage(ch101_dist_pub);
ch101_norm_msg = rosmessage(ch101_norm_pub);
ch101_dist_msg.Data = [nan, nan, nan, nan];
ch101_norm_msg.Data = [nan, nan, nan];

%% 
port = 'COM8';
n_sensors = 4;
if exist('sensors','var')
    clear sensors
end
sensors = serialport(port,9600,'DataBits',8,'Parity','none','StopBits',1);
dist = zeros(1,n_sensors);
buffer_size = 22; dist_buffer = [];
port_flag = zeros(1,n_sensors,'logical');
freq = 20;
rate = rateControl(freq);
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
    % filtering
    dist_buffer = [dist_buffer; dist];
    if length(dist_buffer) > buffer_size
        dist_buffer(1,:) = [];
    end
    dist_filtered = FilterRawDist(dist_buffer);
    % get surface normal
    norm = GetSurfNorm(dist_filtered); % amplify magnitude  
    tilt = max(min(dot([0,0,-1],norm)/(1*vecnorm(norm)),1),-1);
    % vis
    fprintf('port0: %f[mm]\tport1: %f[mm]\tport2: %f[mm]\tport3: %f[mm]\n',dist(1),dist(2),dist(3),dist(4))
    fprintf('tilted angle: %f [deg]\n',real(acosd(tilt)))
    % publish message
    ch101_dist_msg.Data = dist;
    ch101_norm_msg.Data = norm;
    send(ch101_dist_pub, ch101_dist_msg);
    send(ch101_norm_pub, ch101_norm_msg);
    waitfor(rate);
    toc;
end