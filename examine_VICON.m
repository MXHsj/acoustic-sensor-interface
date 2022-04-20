%% process VICON captured data
clc; clear; close all

%% flat surface
surf_st_comp = readtable('data/02-20-2022-flat_surface-static-comp.csv');
surf_st_nocomp = readtable('data/02-20-2022-flat_surface-static-nocomp.csv');
surf_dy_comp = readtable('data/02-20-2022-flat_surface-dynamic-comp.csv');
surf_dy_nocomp = readtable('data/02-20-2022-flat_surface-dynamic-nocomp.csv');
surf_st_nocomp_samples = length(surf_st_nocomp.err_angle);
surf_st_comp_samples = length(surf_st_comp.err_angle);
surf_dy_nocomp_samples = length(surf_dy_nocomp.err_angle);
surf_dy_comp_samples = length(surf_dy_comp.err_angle);

figure('Position',[1920/3, 1080/3, 1200, 600])
subplot(2,2,1)
plot((1:surf_st_nocomp_samples)./100, surf_st_nocomp.err_angle,'--m','LineWidth',1); hold on
ylim([0 50])
xlabel('time (sec)'); ylabel('error (deg)')
title('w/o compensator static tracking')
ax = gca; ax.LineWidth = 1.5; ax.YGrid = 'on';

% subplot(2,2,2)
plot((1:surf_st_comp_samples)./100, surf_st_comp.err_angle,'Color','k','LineWidth',2)
ylim([0 50])
xlabel('time (sec)'); ylabel('error (deg)')
title('w/ compensator static tracking')
ax = gca; ax.LineWidth = 1.5; ax.YGrid = 'on';

subplot(2,2,3)
plot((1:surf_dy_nocomp_samples)./100, surf_dy_nocomp.err_angle,'--m','LineWidth',1); hold on
ylim([0 40])
xlabel('time (sec)'); ylabel('error (deg)')
title('w/o compensator dynamic tracking')
ax = gca; ax.LineWidth = 1.5; ax.YGrid = 'on';

% subplot(2,2,4)
plot((1:surf_dy_comp_samples)./100, surf_dy_comp.err_angle,'Color','k','LineWidth',2)
ylim([0 40])
xlabel('time (sec)'); ylabel('error (deg)')
title('w/ compensator dynamic tracking')
ax = gca; ax.LineWidth = 1.5; ax.YGrid = 'on';

%% mannequin
body_st_comp1 = readtable('data/02-20-2022-body-static-comp1.csv');
body_st_comp2 = readtable('data/02-20-2022-body-static-comp2.csv');
body_st_comp3 = readtable('data/02-20-2022-body-static-comp3.csv');

body_st_comp1_samples = length(body_st_comp1.err_angle);
body_st_comp2_samples = length(body_st_comp2.err_angle);
body_st_comp3_samples = length(body_st_comp3.err_angle);

errors = zeros(12,3);
errors(:,1) = body_st_comp1.err_angle;
errors(:,2) = body_st_comp2.err_angle;
errors(:,3) = body_st_comp3.err_angle;

figure('Position',[1920/3, 1080/3, 400, 500])
layout = tiledlayout(4,1);
ylabel(layout,'error [deg]'); xlabel(layout,'target ID')

% ===== target 1-3 =====
ax1 = nexttile;
boxchart(errors(1:3,:),'BoxFaceColor','#77AC30'); box on; hold on; ylim([0 40]);
plot(median(errors(1:3,:),1),'.-','Color',[48,55,61]./255,'MarkerSize',15)
% legend('error data', 'mean error','NumColumns',2)
set(ax1, 'XTickLabel', num2cell(1:3))
% bar(1:3,mean(errors(1:3,:),2)); ylim([0 30])
ax1.YGrid = 'on'; ax1.LineWidth = 1.0;
% ===== target 4-6 =====
ax2 = nexttile;
boxchart(errors(4:6,:),'BoxFaceColor','#77AC30'); box on; hold on; ylim([0 40])
plot(median(errors(4:6,:),1),'.-','Color',[48,55,61]./255,'MarkerSize',15)
% legend('error data', 'mean error','NumColumns',2)
set(ax2, 'XTickLabel', num2cell(4:6))
% bar(4:6,mean(errors(4:6,:),2)); ylim([0 30])
ax2.YGrid = 'on'; ax2.LineWidth = 1.0;
% ===== target 7-9 =====
ax3 = nexttile;
boxchart(errors(7:9,:),'BoxFaceColor','#77AC30'); box on; hold on; ylim([0 40])
plot(median(errors(7:9,:),1),'.-','Color',[48,55,61]./255,'MarkerSize',15)
% legend('error data', 'mean error','NumColumns',2)
set(ax3, 'XTickLabel', num2cell(7:9))
% bar(7:9,mean(errors(7:9,:),2)); ylim([0 30])
ax3.YGrid = 'on'; ax3.LineWidth = 1.0;
% ===== target 10-12 =====
ax4 = nexttile;
boxchart(errors(10:12,:),'BoxFaceColor','#77AC30'); box on; hold on; ylim([0 40])
plot(median(errors(10:12,:),1),'.-','Color',[48,55,61]./255,'MarkerSize',15)
% legend('error data', 'mean error','NumColumns',2)
set(ax4, 'XTickLabel', num2cell(10:12))
% bar(10:12,mean(errors(10:12,:),2)); ylim([0 30])
ax4.YGrid = 'on'; ax4.LineWidth = 1.0;

%% flat surface animation -- surface static w/o compensator
h = figure();
aviObj = VideoWriter('surf_st_nocomp','MPEG-4');
aviObj.FrameRate = surf_st_nocomp_samples/61; aviObj.Quality = 100;
open(aviObj);
plot(1:surf_st_nocomp_samples, surf_st_nocomp.err_angle,'Color','#5671ba','LineWidth',1); hold on
grid on
ylim([0 50])
xlabel('sample'); ylabel('error [deg]')
title('w/o compensator static tracking')
for i = 1:surf_st_nocomp_samples
    curr_pnt = plot(i,surf_st_nocomp.err_angle(i),'.r','MarkerSize',25);
    annotate = text(i-10,surf_st_nocomp.err_angle(i)+1.5,num2str(surf_st_nocomp.err_angle(i)));
    annotate.FontSize = 12;
    pause(0.0001)
    writeVideo(aviObj, getframe(h));
    if i < surf_st_nocomp_samples
        delete(curr_pnt); delete(annotate)
    end
end
close(aviObj);

%% flat surface animation -- surface static w compensator
h = figure();
aviObj = VideoWriter('surf_st_comp','MPEG-4');
aviObj.FrameRate = surf_st_comp_samples/61; aviObj.Quality = 100;
open(aviObj);
plot(1:surf_st_comp_samples, surf_st_comp.err_angle,'Color','#5671ba','LineWidth',1); hold on; grid on
ylim([0 50])
xlabel('sample'); ylabel('error [deg]')
title('w/ compensator static tracking')
for i = 1:surf_st_comp_samples
    curr_pnt = plot(i,surf_st_comp.err_angle(i),'.r','MarkerSize',25);
    annotate = text(i-10,surf_st_comp.err_angle(i)+1.5,num2str(surf_st_comp.err_angle(i)));
    annotate.FontSize = 12;
    pause(0.0001)
    writeVideo(aviObj, getframe(h));
    if i < surf_st_comp_samples
        delete(curr_pnt); delete(annotate)
    end
end
close(aviObj);

%% flat surface animation -- surface dynamic w/o compensator
h = figure();
aviObj = VideoWriter('surf_dy_nocomp','MPEG-4');
aviObj.FrameRate = surf_dy_nocomp_samples/42; aviObj.Quality = 100;
open(aviObj);
plot(1:surf_dy_nocomp_samples, surf_dy_nocomp.err_angle,'Color','#5671ba','LineWidth',1); hold on; grid on
ylim([0 50])
xlabel('sample'); ylabel('error [deg]')
title('w/o compensator dynamic tracking')
for i = 1:surf_dy_nocomp_samples
    curr_pnt = plot(i,surf_dy_nocomp.err_angle(i),'.r','MarkerSize',25);
    annotate = text(i-10,surf_dy_nocomp.err_angle(i)+1.5,num2str(surf_dy_nocomp.err_angle(i)));
    annotate.FontSize = 12;
    pause(0.0001)
    writeVideo(aviObj, getframe(h));
    if i < surf_dy_nocomp_samples
        delete(curr_pnt); delete(annotate)
    end
end
close(aviObj);

%% flat surface animation -- surface dynamic w compensator
h = figure();
aviObj = VideoWriter('surf_dy_comp','MPEG-4');
aviObj.FrameRate = surf_dy_comp_samples/46; aviObj.Quality = 100;
open(aviObj);
plot(1:surf_dy_comp_samples, surf_dy_comp.err_angle,'Color','#5671ba','LineWidth',1); hold on; grid on
ylim([0 50])
xlabel('sample'); ylabel('error [deg]')
title('w compensator dynamic tracking')
for i = 1:surf_dy_comp_samples
    curr_pnt = plot(i,surf_dy_comp.err_angle(i),'.r','MarkerSize',25);
    annotate = text(i-10,surf_dy_comp.err_angle(i)+1.5,num2str(surf_dy_comp.err_angle(i)));
    annotate.FontSize = 12;
    pause(0.0001)
    writeVideo(aviObj, getframe(h));
    if i < surf_dy_comp_samples
        delete(curr_pnt); delete(annotate)
    end
end
close(aviObj);