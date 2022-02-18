%% real-time sensor compensation
function [correct] = CompSensorRt(measure, sensor_ID, sensor_mean)

correct=[]; num=0;

% get the nth sensor mean measurement
real=sensor_mean(sensor_ID,:);

% find index and replace with correct measurement
[~,Index] = min(abs(real-measure));
num=(Index+3)*10;

correct = num;

