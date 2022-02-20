%% process sensor_test data
clc; clear; close all
load('data/com_sensor_test.mat');
load('data/sensor_test.mat');

% ignore first 10 and last 1 measurements, consider 4 to 20 cm range (index 3 to 19)
sensor1_valid = sensor_1(10:end-1,3:end);
sensor2_valid = sensor_2(10:end-1,3:end);
sensor3_valid = sensor_3(10:end-1,3:end);
sensor4_valid = sensor_4(10:end-1,3:end);
sensor_valid = zeros([size(sensor1_valid),sensor_num]);
sensor_valid(:,:,1) = sensor1_valid;
sensor_valid(:,:,2) = sensor2_valid;
sensor_valid(:,:,3) = sensor3_valid;
sensor_valid(:,:,4) = sensor4_valid;
%after compensator
com_sensor1_valid = com_sensor_1(10:end-10,3:end);
com_sensor2_valid = com_sensor_2(10:end-10,3:end);
com_sensor3_valid = com_sensor_3(10:end-10,3:end);
com_sensor4_valid = com_sensor_4(10:end-10,3:end);
com_sensor_valid = zeros([size(com_sensor1_valid),sensor_num]);
com_sensor_valid(:,:,1) = com_sensor1_valid;
com_sensor_valid(:,:,2) = com_sensor2_valid;
com_sensor_valid(:,:,3) = com_sensor3_valid;
com_sensor_valid(:,:,4) = com_sensor4_valid;

% calculate mean error
ground_truth = 40:10:200;
sensor1_merr = ground_truth - mean(sensor1_valid);
sensor2_merr = ground_truth - mean(sensor2_valid);
sensor3_merr = ground_truth - mean(sensor3_valid);
sensor4_merr = ground_truth - mean(sensor4_valid);
sensor_merr = zeros(4,length(sensor1_merr));
sensor_merr(1,:) = sensor1_merr;
sensor_merr(2,:) = sensor2_merr;
sensor_merr(3,:) = sensor3_merr;
sensor_merr(4,:) = sensor4_merr;
%after
com_sensor1_merr = ground_truth - mean(com_sensor1_valid);
com_sensor2_merr = ground_truth - mean(com_sensor2_valid);
com_sensor3_merr = ground_truth - mean(com_sensor3_valid);
com_sensor4_merr = ground_truth - mean(com_sensor4_valid);
com_sensor_merr = zeros(4,length(com_sensor1_merr));
com_sensor_merr(1,:) = com_sensor1_merr;
com_sensor_merr(2,:) = com_sensor2_merr;
com_sensor_merr(3,:) = com_sensor3_merr;
com_sensor_merr(4,:) = com_sensor4_merr;


%calculate mean measurement
sensor_mean = zeros(4,length(sensor1_merr));
sensor_mean(1,:) = mean(sensor1_valid);
sensor_mean(2,:) = mean(sensor2_valid);
sensor_mean(3,:) = mean(sensor3_valid);
sensor_mean(4,:) = mean(sensor4_valid);
sensor_mean_sim=mean(sensor_mean,1);
%after
com_sensor_mean = zeros(4,length(com_sensor1_merr));
com_sensor_mean(1,:) = mean(com_sensor1_valid);
com_sensor_mean(2,:) = mean(com_sensor2_valid);
com_sensor_mean(3,:) = mean(com_sensor3_valid);
com_sensor_mean(4,:) = mean(com_sensor4_valid);
com_sensor_mean_sim=mean(com_sensor_mean,1);

% calculate standard deviation
sensor_std = zeros(4,length(sensor1_merr));
sensor_std(1,:) = std(sensor1_valid);
sensor_std(2,:) = std(sensor2_valid);
sensor_std(3,:) = std(sensor3_valid);
sensor_std(4,:) = std(sensor4_valid);
%after
com_sensor_std = zeros(4,length(com_sensor1_merr));
com_sensor_std(1,:) = std(com_sensor1_valid);
com_sensor_std(2,:) = std(com_sensor2_valid);
com_sensor_std(3,:) = std(com_sensor3_valid);
com_sensor_std(4,:) = std(com_sensor4_valid);

%% visualize error distribution
for sensor = 1:sensor_num
    figure('Name',['sensor',num2str(sensor)],'Position',[1920/2,1080/3,600*2,500])
    sensor_err = ground_truth - sensor_valid(:,:,sensor);
    
    for measure = 1:size(sensor_err,2)
        subplot(3,6,measure)
        histogram(sensor_err(:,measure),'Normalization','pdf')
        hold on
        err = min(sensor_err(:,measure)):0.1:max(sensor_err(:,measure));
        mu = mean(sensor_err(:,measure));
        sigma = std(sensor_err(:,measure));
        f = exp(-(err-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi)); % fit to Gaussian
        plot(err,f,'LineWidth',1.5)
        xlabel('error [mm]'), ylabel('normalized probability');
        title(['@',num2str(ground_truth(measure)),' [mm]'])
    end
end

%% visualize sensor accuracy
sensor_num=4
figure('Position',[1920/2,1080/3,800,600])
working_dist = [50, 50, 120, 120];
error_range = [-45,45,45,-45];
for sensor = 1:sensor_num
    subplot(2,2,sensor)
    errorbar(ground_truth,sensor_merr(sensor,:),sensor_std(sensor,:))
    grid on 
    hold on
    errorbar(ground_truth,com_sensor_merr(sensor,:),com_sensor_std(sensor,:))
    grid on
    hold on
    patch(working_dist,error_range,'red','FaceAlpha',.15,'EdgeColor','none')
    xlim([ground_truth(1),ground_truth(end)]); ylim(error_range(1:2));
    xticks(linspace(ground_truth(1),ground_truth(end),5))
    ylabel('error [mm]'); xlabel('distance [mm]')
    title(['sensor',num2str(sensor)])

end
% average
figure()
p_mean = polyfit(ground_truth, mean(sensor_merr),4);
p_std = polyfit(ground_truth, mean(sensor_std),4);
% plot(ground_truth, polyval(p_mean,ground_truth),'LineWidth',1)


%% error compensation
figure('Position',[1920/2,1080/3,800,600])
working_dist = [50, 50, 120, 120];
error_range = [-40,40,40,-40];
for sensor = 1:sensor_num
    subplot(2,2,sensor)
    p_err = polyfit(mean(sensor_valid(:,:,sensor)),sensor_merr(sensor,:),4);
    plot(mean(sensor_valid(:,:,sensor)),sensor_merr(sensor,:))
    grid on 
    hold on
    plot(mean(sensor_valid(:,:,sensor)),polyval(p_err,mean(sensor_valid(:,:,sensor))));
    patch(working_dist,error_range,'red','FaceAlpha',.15,'EdgeColor','none')
    axis tight
    ylim(error_range(1:2))
    ylabel('error [mm]'); xlabel('sensor measurement [mm]')
    title(['sensor',num2str(sensor)])
    legend('error','fitted error')
end
% get averaged error vs. measurement for 4 sensors
p_err_mean = polyfit(mean(mean(sensor_valid),3),sensor_merr(sensor,:),4);
