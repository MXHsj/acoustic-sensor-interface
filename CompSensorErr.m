%% ====================================================
% file name:    CompSensorErr.m
% author:       Wen-Yi Kuo
% description:  compensate sensor error
% input:        
% output:       corrected sensor measurements
% =====================================================
%vq = interp1(x,v,xq,method)
%
function correct=CompSensorErr(measure,sensor_mean)

correct=[]; num=0;
%get the nth sensor mean measurement
%real=sensor_mean(nth_sensor,:);
v=40:10:200;
for i=1:length(measure)
    xq=measure(i);
    x=sensor_mean(i,:);

    vq = interp1(x,v,xq);
    if isnan(vq)
        vq=200;
    end
    %find index and replace with correct measurement
    %[~,Index] = min(abs(sensor_mean(i,:)-measure(i)));
    %num=(Index+3)*10;

    correct=[correct;vq];
end
