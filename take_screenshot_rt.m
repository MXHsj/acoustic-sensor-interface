%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% file name: take_screenshot_rt.m
% author: Xihan Ma
% description: get realtime screenshots of selected window (teraterm)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
clc; clear; close all

% Take screen capture
left = 130; top = 494;
width = round(1048 * 0.4);   % half screen width
height = round(968 * 0.23);
robot = java.awt.Robot();
pos = [left top width height]; % [left top width height]
rect = java.awt.Rectangle(pos(1),pos(2),pos(3),pos(4));
winCap_rgb = zeros(height,width,3,'uint8');

while 1
    cap = robot.createScreenCapture(rect);
    % convert to an RGB image
    rgb = typecast(cap.getRGB(0,0,cap.getWidth,cap.getHeight,[],0,cap.getWidth),'uint8');
    winCap_rgb(:,:,1) = reshape(rgb(3:4:end),cap.getWidth,[])';
    winCap_rgb(:,:,2) = reshape(rgb(2:4:end),cap.getWidth,[])';
    winCap_rgb(:,:,3) = reshape(rgb(1:4:end),cap.getWidth,[])';
    imshow(winCap_rgb)
    
    txt = ocr(winCap_rgb);
    disp(txt.Words)
end