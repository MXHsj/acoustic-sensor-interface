%% read VL53L0X laser range finder values from serial input
clc; clear; close all

%%
port = 'COM7';
% IDs = [238 238 238 238];
IDs = [1 2];
if exist('sensors','var')
    clear sensors
end
sensors = serialport(port,115200,'DataBits',8,'Parity','none','StopBits',1);

port_flag = zeros(1,length(IDs),'logical');
dist = zeros(1,length(IDs));
buffer_size = 20; dist_buffer = [];
n_sample = 200;      % set data_count to -1 to disable recording
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
    % filtering
    dist_buffer = [dist_buffer; dist];
    if length(dist_buffer) > buffer_size
        dist_buffer(1,:) = [];
    end
    dist_filtered = FilterRawDist(dist_buffer);
    % vis
    fprintf('sensor %d range: %f [mm]\n', IDs(1), dist(1))
    fprintf('sensor %d range: %f [mm]\n', IDs(2), dist(2))
%     fprintf('sensor %d range: %f [mm]\n', IDs(3), dist(3))
%     fprintf('sensor %d range: %f [mm]\n', IDs(4), dist(4))
    % record distance and amplitude
    dist_rec(count_sample, :) = dist_filtered;
    count_sample = count_sample + 1;
%     toc;
end

%% plot recorded data
figure('Position',[1920/6,1080/6,1920,720])
plot(1:n_sample, dist_rec(:,1))
ylabel('range [mm]')
hold on; grid on;
plot(1:n_sample, dist_rec(:,2))

