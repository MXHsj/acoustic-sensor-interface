%% ====================================================
% file name:    CompSensorErrRt.m
% author:       Xihan Ma
% description:  compensate sensor error in real-time
% input:        sensor distances (1x4 vector)
% output:       corrected sensor measurements
% =====================================================
function correct = CompSensorErrRt(measure, useSameSensor)

p_err1 = [0.0, -0.0003, 0.0539, -4.2129, 96.6131];
p_err2 = [0.0, -0.0003, 0.0395, -2.4977, 45.6929];
p_err3 = [0.0, 0.0001, -0.0242, 1.6273, -41.7986];
p_err4 = [0.0, -0.0004, 0.0539, -2.9106, 59.4630];
p_err_mean = [6.2910e-7, -2.5087e-4, 0.0394, -2.7782, 74.8601];

if useSameSensor    % use the same error model for all sensors (for simulation)
    correct = zeros(size(measure));
    for i = 1:length(measure)
        correct(i) = measure(i) + polyval(p_err_mean, measure(i));
    end
else
    if length(measure) ~= 4
        correct = nan(size(measure));
        disp('not enough sensor measurements')
        return
    else
        correct1 = measure(1) + polyval(p_err1, measure(1));
        correct2 = measure(2) + polyval(p_err2, measure(2));
        correct3 = measure(3) + polyval(p_err3, measure(3));
        correct4 = measure(4) + polyval(p_err4, measure(4));
        correct = [correct1, correct2, correct3, correct4];
    end    
end

% apply no correction if measurement greater than 200 mm
for i = 1:length(measure)
    if correct(i) > 200
%         correct(i) = measure(i);
        correct(i) = 200;
%         correct(i) = nan;
    end
end