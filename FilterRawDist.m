%% ====================================================
% file name:    FilterRawDist.m
% author:       Xihan Ma
% description:  filter temporal distance data from ch101
% input:        raw distance in a buffer
% output:       filtered distance in a same-length buffer
% =====================================================
function [dist_filtered] = FilterRawDist(dist_buffer)

dist_buffer = rmoutliers(dist_buffer,'mean');
dist_filtered = mean(dist_buffer,1,'omitnan');


