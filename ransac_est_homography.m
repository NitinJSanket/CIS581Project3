function [H,inlier_ind] = ransac_est_homography(y1, x1, y2, x2, thresh, iterations)
disp(['Performing RANSAC']);
% iterations = 100;
NPts = size(x1,1);

for i = 1:iterations
    % Step 1: Select four feature pairs (at random)
    RandIdxs = randperm(NPts, 4);
    % Step 2: Compute homography H (exact)
    HExact{i} = est_homography(x1(RandIdxs),y1(RandIdxs),x2(RandIdxs),y2(RandIdxs));
    [XEstimate, YEstimate] = apply_homography(HExact{i}, x2, y2);
    % Step 3: Compute inliers where SSD(pi�, H pi) < thresh
    Inliers{i} = ((x1 - XEstimate).^2 + (y1 - YEstimate).^2) <= thresh.^2;
    Votes(i) = sum(Inliers{i});
end
% Step 4: Find te set of inliers with the highest votes
% Find the best score
[~, MaxVoted] = max(Votes);
Inliers = Inliers{MaxVoted};
inlier_ind = find(Inliers)';
H = est_homography(x1(Inliers),y1(Inliers),x2(Inliers),y2(Inliers));
end
