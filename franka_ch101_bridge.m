%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file name: franka_ch101_bridge.m
% author: Xihan Ma
% description: get ch101 reading from screenshot, send to franka via ROS
% dependency: robotics tool box
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
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
ch101_msg.data = nan;

%% get ch101 reading
% define ROI
screen_width = 1920; screen_height = 1080;
left = 0; top = 400;
width = round(0.8*screen_width/2);
height = round(0.7*(screen_height-top));
robot = java.awt.Robot();
pos = [left top width height]; % [left top width height]
rect = java.awt.Rectangle(pos(1),pos(2),pos(3),pos(4));
winCap_rgb = zeros(height,width,3,'uint8');

while true
    tic
    cap = robot.createScreenCapture(rect);
    % convert to an RGB image
    rgb = typecast(cap.getRGB(0,0,cap.getWidth,cap.getHeight,[],0,cap.getWidth),'uint8');
    winCap_rgb(:,:,1) = reshape(rgb(3:4:end),cap.getWidth,[])';
    winCap_rgb(:,:,2) = reshape(rgb(2:4:end),cap.getWidth,[])';
    winCap_rgb(:,:,3) = reshape(rgb(1:4:end),cap.getWidth,[])';
%     imshow(winCap_rgb)
    
    txt = ocr(winCap_rgb,'TextLayout','Block','CharacterSet','');
    port0_indC = strfind(txt.Words,'0:');
    port0_ind = find(not(cellfun('isempty',port0_indC)));
    port0_range = zeros(1,length(port0_ind));
    for i  = 1:length(port0_ind)
        port0_range(i) = str2double(txt.Words{port0_ind(i)+2});
    end
    % publish message
    fprintf('port0 range: [mm]: %f \t',port0_range(end))
    ch101_msg.data = port0_range(end);
    send(ch101_pub, ch101_msg);
    toc
end

%% visualization
figure()
plot(1:length(port0_range), port0_range,'-*b')
ylim([0.98*max(port0_range), 1.02*max(port0_range)])
ylabel('distance [mm]')
ax = gca(); ax.YGrid = 'on';

