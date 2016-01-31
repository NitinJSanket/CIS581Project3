%% Code to perform Automatic Image Stitching
% Project 3 for CIS 581: Computer Vision and Computational Photography
% Written by: 
% Nitin J. Sanket (nitinsan@seas.upenn.edu)
% First Year MSE in Robotics Student
% University of Pennsylvania
%% Feature Matching
function [m] = feat_match(p1,p2, Thld)
m = -ones(size(p1, 2) ,1); % Initialize m to -1
% Thld = 0.65; % Threshold for goodness of a feature

for i = 1:size(p1,2)
    % Compute SSD between all pairs of 8x8 descriptors
    % in one image to descriptors in second image
    SSD = sum(bsxfun(@minus, p1(:,i), p2).^2, 1);
    % For each corner, find the 2 nearest neighbors
    % (using SSD from above as the distance) in the second image
    [SSDSorted, SSDIdx] = sort(SSD, 2, 'ascend');
    % Find ratio of SSD(1) to SSD(2)
    SSDRatio = SSDSorted(1)/SSDSorted(2);
%     disp(SSDRatio);
    if(SSDRatio<Thld)
        m(i) = SSDIdx(1);
    end
end
end