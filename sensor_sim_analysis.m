%% process sensor simulation data
clc; clear; close all

err_rec = [];
for i = 3:8
    load(['data/sensor_sim{n=',num2str(i),'}.mat'])
    err_rec = [err_rec; evalin('base',['err_rec_',num2str(i)])];
end

% scatter error
figure('Position',[1920/3, 1080/4, 1200, 600])
for i = 1:size(err_rec,1)
    subplot(2,3,i)
    plot(1:length(err_rec(i,:)),abs(err_rec(i,:)),'.');
    xlabel('sample'); ylabel('absolute error [deg]');
    title([num2str(i+2),' sensors'])
    ylim([0, 25])
    grid on
end

% performance vs. # of sensors

