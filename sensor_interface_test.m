clc;
clear;
close all

port = 'COM3';
s = serialport(port,10000,'DataBits',8,'Parity','even');

data = read(s,10,"double");
data

% fopen(s);
% msg = fscanf(s);
% disp(msg)
% fclose(s);
% delete(s);
% clear s;
