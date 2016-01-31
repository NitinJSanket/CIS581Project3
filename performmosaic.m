%% Code to perform Automatic Image Stitching
% Project 3 for CIS 581: Computer Vision and Computational Photography
% Written by: 
% Nitin J. Sanket (nitinsan@seas.upenn.edu)
% First Year MSE in Robotics Student
% University of Pennsylvania
%% Code to Perform Mosaicing
function img_mosaic = performmosaic(I1, I2, NCorners, ClippingFlag, RANSACThld, RANSACIteration, ClippingPercentage);
disp(['Performing Image Mosaicing']);

%% Get CornerMetric Matrix
C1 = cornermetric(rgb2gray(I1), 'harris'); % Corner Metric matrix
C2 = cornermetric(rgb2gray(I2), 'harris'); % Corner Metric matrix

%% ANMS
tic
[y1, x1, rmax1] = anms(C1, NCorners, ClippingFlag, ClippingPercentage);
[y2, x2, rmax2] = anms(C2, NCorners, ClippingFlag, ClippingPercentage);
toc
figure,
imshow(I1);
hold on;
plot(x1, y1,'r.'); % Plot is always (x,y)
figure,
imshow(I2);
hold on;
plot(x2, y2,'r.'); % Plot is always (x,y)
%% Feature Descriptor
% Feature descriptors
p1 = feat_desc(I1, y1, x1);
p2 = feat_desc(I2, y2, x2);

%% Feature matching
m  = feat_match(p1,p2, 0.75);
Y1 = y1(m ~= -1);
X1 = x1(m ~= -1);
Y2 = y2(m(m ~= -1));
X2 = x2(m(m ~= -1));

X1A = X1;
X2A = X2;
Y1A = Y1;
Y2A = Y2;
% figure,
% showMatchedFeatures(I1,I2,[X1 Y1],[X2 Y2],'montage');%, 'PlotOptions', {'b.', 'b.', 'b-'});
% pause;
%% RANSAC with output shown
[H, inlier_ind12] = ransac_est_homography(Y1, X1, Y2, X2, RANSACThld, RANSACIteration);
Y1 = Y1(inlier_ind12');
X1 = X1(inlier_ind12');
Y2 = Y2(inlier_ind12');
X2 = X2(inlier_ind12');
figure,
showMatchedFeatures(I1,I2,[X1 Y1],[X2 Y2],'montage');%, 'PlotOptions', {'b.', 'b.', 'b-'});
% pause;
% hold on;
% plot(X1A, Y1A, 'r.');
% plot(X2A, Y2A, 'r.');
% hold off;

%% Calculate Blend Fraction from Histograms
[H1, H1x] = imhist(rgb2gray(I1));
H1 = H1./max(H1);
% figure,
% stem(H1x, H1);

[H2, H2x] = imhist(rgb2gray(I2));
H2 = H2./max(H2);
% figure,
% stem(H2x, H2);

H1Mean = mean(H1);
H2Mean = mean(H2);

blendFrac = H1Mean./(H1Mean+H2Mean);
% Reference for this part:
% http://www.leet.it/home/giusti/teaching/matlab_sessions/stitching/stitch.html
%% Bounding Box for warped source Image
[m1, n1, ~] = size(I1); % Destination Image Y,X
[m2, n2, ~] = size(I2); % Source Image Y,X


% Apply Homography on corner points of the destination image to find the limits
[mLimits, nLimits] = apply_homography(H, [1, 1, n2, n2]', [1, m2, 1, m2]');
% Calculate the upper and lower limits
LowerLimit = round(min([mLimits, nLimits], [], 1));
UpperLimit = round(max([mLimits, nLimits], [], 1));


% Calculate all values of transformed x's and y's
[xTrans, yTrans] = meshgrid(LowerLimit(1):UpperLimit(1), LowerLimit(2):UpperLimit(2));
% Reshape both x and y to get a column vector
xTrans1 = reshape(xTrans, [numel(xTrans), 1]);
yTrans1 = reshape(yTrans, [numel(yTrans), 1]);


% Apply Inverse Homography on transformed x's and y's to get the
% corresponding source x's and y's
[xSource, ySource] = apply_homography(inv(H), xTrans1, yTrans1);
xSource = round(xSource);
ySource = round(ySource);


%% Bounding Box for Stictched Image
LowerLimitStitch = min([LowerLimit; [1 1]], [], 1);
UpperLimitStitch = max([UpperLimit; [n1, m1]], [], 1);

img_mosaic = uint8(zeros([UpperLimitStitch(2) - LowerLimitStitch(2) + 1, UpperLimitStitch(1) - LowerLimitStitch(1) + 1, 3]));
LowerLimitDestination = 1 - (LowerLimitStitch ~= 1) .* LowerLimitStitch;
img_mosaic(LowerLimitDestination(2):LowerLimitDestination(2)+m1-1, LowerLimitDestination(1):LowerLimitDestination(1)+n1-1,:) = I1;


%% Clip Extroneus pixels
FinalIdx = xSource >= 1 & xSource <= n2 & ySource >= 1 & ySource <= m2;
xTrans1 = xTrans1(FinalIdx);
yTrans1 = yTrans1(FinalIdx);
xSource = xSource(FinalIdx);
ySource = ySource(FinalIdx);


%% Stitch the source image (linear blend)
for i = 1 : length(xTrans1)
if all(img_mosaic(yTrans1(i) - LowerLimitStitch(2) + 1, xTrans1(i) - LowerLimitStitch(1) + 1, :) == 0)
img_mosaic(yTrans1(i) - LowerLimitStitch(2) + 1, xTrans1(i) - LowerLimitStitch(1) + 1, :) = I2(ySource(i), xSource(i), :);
else
img_mosaic(yTrans1(i) - LowerLimitStitch(2) + 1, xTrans1(i) - LowerLimitStitch(1) + 1, :) = ...
    blendFrac*img_mosaic(yTrans1(i) - LowerLimitStitch(2) + 1, xTrans1(i) - LowerLimitStitch(1) + 1, :) ...
    + (1 - blendFrac)*I2(ySource(i), xSource(i), :);
end
end

figure,
imshow(img_mosaic);

end