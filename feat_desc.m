%% Code to perform Automatic Image Stitching
% Project 3 for CIS 581: Computer Vision and Computational Photography
% Written by: 
% Nitin J. Sanket (nitinsan@seas.upenn.edu)
% First Year MSE in Robotics Student
% University of Pennsylvania
%% Feature Descriptor
function [p] = feat_desc(im, y, x)
disp(['Performing Feature Description']);
im = im2double(padarray(im, [20, 20]));
NoPatches = size(x, 1);
p  = zeros(64, NoPatches);
h = fspecial('gaussian'); % Gaussian Kernel for smoothing

for i = 1:NoPatches
    Patch = im(y(i):y(i)+39, x(i):x(i)+39);
    Patch = imfilter(Patch, h, 'same'); % Gaussian Blur the 40x40 patch
    Patch = Patch(1:5:40, 1:5:40); % Subsample the 40x40 patch to get 8x8 patch
    Subsampled = reshape(Patch, [64, 1]);
    Subsampled = Subsampled - mean(Subsampled); % Make mean = 0
    Subsampled = Subsampled./std(Subsampled,1); % Make SD = 1
    p(:,i) = Subsampled;
end
end