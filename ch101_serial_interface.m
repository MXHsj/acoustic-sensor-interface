%% read ch101 sensor values from serial input
clc; clear; close all

%% 
port = 'COM4';
n_sensors = 4;
if exist('sensors','var')
    clear sensors
end
sensors = serialport(port,9600,'DataBits',8,'Parity','none','StopBits',1);
port_flag = zeros(1,n_sensors,'logical');
dist = zeros(1,n_sensors); amp = zeros(1,n_sensors);
n_sample = 400;      % set data_count to -1 to disable recording
dist_rec = zeros(n_sample,n_sensors); amp_rec = zeros(n_sample,n_sensors);
count_sample = 1;

while n_sample > 0 && count_sample < n_sample
    tic
    latest = char(readline(sensors));
    % check which port
    for i=1:n_sensors
        port_flag(i) = contains(latest,[num2str(i-1),':']);
    end
    % extract range
    range_ind = strfind(latest, 'Range: ');
    if ~isempty(range_ind)
        dist(port_flag) = str2double(latest(range_ind+length('Range: '):range_ind+length('Range: ')+4));
    else
        dist(port_flag) = nan;
    end
    % extract amplitude
    amp_ind = strfind(latest, 'Amp: ');
    if ~isempty(amp_ind)
        amp(port_flag) = str2double(latest(amp_ind+length('Amp: '):amp_ind+length('Amp: ')+4));
    else
        amp(port_flag) = nan;
    end
    % vis
    fprintf('port0 range: %f [mm]\t amp: %f \n', dist(1), amp(1))
    fprintf('port1 range: %f [mm]\t amp: %f \n', dist(2), amp(2))
    fprintf('port2 range: %f [mm]\t amp: %f \n', dist(3), amp(3))
    fprintf('port3 range: %f [mm]\t amp: %f \n', dist(4), amp(4))
    % record distance and amplitude
    dist_rec(count_sample, :) = dist;
    amp_rec(count_sample, :) = amp;
    count_sample = count_sample + 1;
    toc;
end

%% plot recorded data
figure('Position',[1920/6,1080/6,1920,720])
yyaxis left
plot(1:n_sample, dist_rec(:,1))
ylabel('range [mm]')
hold on; grid on;
yyaxis right
plot(1:n_sample, amp_rec(:,1))
xlabel('time stamp')
ylabel('amplitude')

