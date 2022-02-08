%% ====================================================
% file name:    GenUnitVector.m
% author:       Xihan Ma
% description:  generate n unit vector
% input:        number of vectors
% output:       unit vector
% =====================================================
function [norm] = GenUnitVector(num, isRandom)
% isRandom = false;

norm = zeros(num,3);
z_min = 0.7; 
z_range = 0.28;
x_min = -0.15;
x_range = 0.3;

if isRandom
    for i = 1:num
        rand_vec(3) = z_min + z_range*rand(1);
        rand_vec(1) = x_min + x_range*rand(1);
        rand_vec(2) = (-1)^randi([1,10],1)*sqrt(1-rand_vec(3)^2-rand_vec(1)^2);
        norm(i,:) = rand_vec;
    end
else
    z = linspace(z_min, z_min+z_range, num);
    x = linspace(x_min, x_min+x_range, num);
    for i = 1:num
        rand_vec(3) = z(i);
        rand_vec(1) = x(i);
        rand_vec(2) = sqrt(1-rand_vec(3)^2-rand_vec(1)^2);
        norm(i,:) = rand_vec;
    end
end
