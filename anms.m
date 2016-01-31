%% Code to perform Automatic Image Stitching
% Project 3 for CIS 581: Computer Vision and Computational Photography
% Written by: 
% Nitin J. Sanket (nitinsan@seas.upenn.edu)
% First Year MSE in Robotics Student
% University of Pennsylvania
%% Code to perform Adaptive Non Maximal Suppression
function [y, x, rmax] = anms(cimg, max_pts, ClippingFlag, ClippingPercentage)
disp(['Performing Adaptive Non Maximum Suppression']);
RegMax = imregionalmax(cimg); % This has all the corners
CornerVal = bsxfun(@times, RegMax, cimg); % Does RegMax.*I 2 times faster
% MaxVal = max(max(CornerVal));
% if(ClippingFlag)
% CornerVal = CornerVal.*(CornerVal>=ClippingPercentage*MaxVal); % 10% Threshold
% end
% We need to find the non-zero values in CornerVal
CornerIdx = find(CornerVal);
[y, x] = ind2sub(size(cimg), CornerIdx);
% x and y have the indexes corresponding to each corner
NoPts = size(x, 1); % Number of points
Rad = Inf(NoPts, 1); % Initialize all radii to infinity
for i = 1:NoPts % Loop for current point
    for j = 1:NoPts % Loop for neighbouring points
        if(cimg(y(j), x(j))>cimg(y(i), x(i)))
            ED = (y(j)-y(i))^2 + (x(j)-x(i))^2;
            if(ED<Rad(i))
                Rad(i) = ED;
            end
        end
    end
end
% Now Rad has all the radii corresponding to each corner
% Sort them in decending order
[RadVal, RadIdx] = sort(Rad, 'descend');
% Check if number of points asked > number of points obtained
if(length(x)<max_pts)
    max_pts = length(x);
    disp(['Number of Points decreased to ', num2str(length(x))]);
end
x = x(RadIdx(1:max_pts));
y = y(RadIdx(1:max_pts));
rmax = sqrt(RadVal(1:max_pts));
end