%% ====================================================
% file name:    CompSensorErr.m
% author:       Wen-Yi Kuo
% description:  compensate sensor error
% input:        
% output:       corrected sensor measurements
% =====================================================
function correct=CompSensorErr(measure,nth_sensor,sensor_mean)

correct=[]; num=0;
%get the nth sensor mean measurement
real=sensor_mean(nth_sensor,:);
for i=1:length(measure)
    %find index and replace with correct measurement
    [~,Index] = min(abs(real-measure(i)));
    num=(Index+3)*10;
    correct=[num;correct];
end