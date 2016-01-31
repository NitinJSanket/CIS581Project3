%% Code to perform Automatic Image Stitching
% Project 3 for CIS 581: Computer Vision and Computational Photography
% Written by: 
% Nitin J. Sanket (nitinsan@seas.upenn.edu)
% First Year MSE in Robotics Student
% University of Pennsylvania
%% Code to Run the other support Functions (A Demo Script)

clc
clear all
close all

NCorners = 400;
ClippingFlag = 1;
RANSACThld = 5;
RANSACIteration = 200;
ClippingPercentage = 0.08;
% Optimal Parameter Settings for Various Datasets
% Set 1: NCorners 400 and RANSACThld 10 RANSACIterations 10000 0.08
% ClippingPercentage
% Set 2: NCorners 200 and RANSACThld 25 RANSACIterations 10000 no clipping
% in ANMS
% Set 3: NCorners 800 and RANSACThld 10 RANSACIterations 10000 no clipping
% in ANMS

warning off;
ImagePath = [pwd, '/Images/Set1/'];
FileNames = dir(fullfile(ImagePath, '*.jpg')); % Reads all .jpg Files
NumImages = numel(FileNames);

for i = 1:NumImages
    I{i} = imread(fullfile(ImagePath, FileNames(i).name));
    disp(['Reading Image File ', num2str(i)]);
%     figure,
%     imshow(I{i});
end

img_mosaic = mymosaic(I, NCorners, ClippingFlag, RANSACThld, RANSACIteration, ClippingPercentage);
disp(['Image Stitching Complete']);
figure,
imshow(img_mosaic);

