%% Code to perform Automatic Image Stitching
% Project 3 for CIS 581: Computer Vision and Computational Photography
% Written by: 
% Nitin J. Sanket (nitinsan@seas.upenn.edu)
% First Year MSE in Robotics Student
% University of Pennsylvania
%% Code to call mosaicing functions
function img_mosaic = mymosaic(img_input, NCorners, ClippingFlag, RANSACThld, RANSACIteration, ClippingPercentage);

for i = 2:size(img_input, 2) % For all images
    disp(['Analyzing Images ', num2str(i-1), ' and ', num2str(i)]); 
    if i == 2 % First case hard coded
        img_mosaic = performmosaic(img_input{1}, img_input{2}, NCorners, ClippingFlag, RANSACThld, RANSACIteration, ClippingPercentage);
    else
        img_mosaic = performmosaic(img_mosaic, img_input{i}, NCorners, ClippingFlag, RANSACThld, RANSACIteration, ClippingPercentage);
    end
    
end
end