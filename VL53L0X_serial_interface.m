%% read VL53L0X laser range finder values from serial input
clc; clear; close all
sensor_1=[];sensor_2=[];sensor_3=[];sensor_4=[];
com_sensor_4=[];

load('data/sensor_mean.mat');

%%
port = 'COM5';
com=[];
IDs = [1 2 3 4];
if exist('sensors','var')
    clear sensors
end
sensors = serialport(port,9600,'DataBits',8,'Parity','none','StopBits',1);

port_flag = zeros(1,length(IDs),'logical');
dist = zeros(1,length(IDs));
buffer_size = 5; dist_buffer = [];
n_sample = 301;      % set data_count to -1 to disable recording
dist_rec = zeros(n_sample,length(IDs));
count_sample = 1;

while n_sample > 0 && count_sample < n_sample
%     tic
    latest = char(readline(sensors));
    % check sensor ID
    for i = 1:length(IDs)
        port_flag(i) = contains(latest,[num2str(IDs(i)),':']);
    end
    % extract range
    range_ind = strfind(latest, 'Range: ');
    if ~isempty(range_ind)
        dist(port_flag) = str2double(latest(range_ind+length('Range: '):range_ind+length('Range: ')+2));
    else
        dist(port_flag) = nan;
    end
    
%     % filtering
%     dist_buffer = [dist_buffer; dist];
%     if length(dist_buffer) > buffer_size
%         dist_buffer(1,:) = [];
%     end
%     dist_filtered = FilterRawDist(dist_buffer);
    % vis
    
%     fprintf('sensor1: %f[mm]\tsensor2: %f[mm]\tsensor3: %f[mm]\tsensor4: %f[mm]\n', ...
%             dist_filtered(1),dist_filtered(2),dist_filtered(3),dist_filtered(4))
%     fprintf('sensor %d range: %f [mm]\n', IDs(1), dist(1))
%     fprintf('sensor %d range: %f [mm]\n', IDs(2), dist(2))
%     fprintf('sensor %d range: %f [mm]\n', IDs(3), dist(3))
%     fprintf('sensor %d range: %f [mm]\n', IDs(4), dist(4))
    % record distance and amplitude
%     dist_rec(count_sample, :) = dist_filtered;
    dist_rec(count_sample, :) = dist;
    correct = CompSensorErr(dist(4),sensor_mean);
    com=[com;correct];
    count_sample = count_sample + 1;
%     toc;
end
   com_sensor_4=[com_sensor_4,com];
   sensor_4=[sensor_4,dist_rec(:,4)];
    %================================
%     test=[10 20 30 40; 20 30 40 50]
% for i = 1:length(IDs)
%     c(:,1)=CompSensorErr(test(i,:),sensor_mean);
%     com=[c,com];
% end
    %================================
   

%% plot recorded data
% figure('Position',[1920/6,1080/6,1920,720])
% plot(1:n_sample, dist_rec(:,1))
% ylabel('range [mm]')
% hold on; grid on;
% plot(1:n_sample, dist_rec(:,2))
%===============change=============
% %measure dis 4cm~20cm
% sensor_num=1;
% %===============change=============
%%
