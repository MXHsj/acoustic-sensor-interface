% clear;clc;
% load('data/sensor_mean.mat');
% load('data/com_sensor_test.mat');
correct=[];
com_sensor_4=[];
measure=sensor_4;
v=40:10:200;
%%
for j=1:19
    for i=1:length(measure)
        xq=measure(i,j);
        x=sensor_mean(4,:);

        vq = interp1(x,v,xq);
        if isnan(vq)
            vq=200;
        end
        correct=[correct;vq];
       
    end
    com_sensor_4=[com_sensor_4,correct];  
    correct=[];
end
