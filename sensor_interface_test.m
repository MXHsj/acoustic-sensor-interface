%% read serial input
clc; clear; close all

port = 'COM3';
s = serialport(port,9600,'DataBits',8,'Parity','none','StopBits',1);

data = read(s,10,"double");
data

% fopen(s);
% msg = fscanf(s);
% disp(msg)
% fclose(s);
% delete(s);
% clear s;
