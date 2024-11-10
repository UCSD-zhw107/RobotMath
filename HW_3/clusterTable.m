% Read Data
data = readmatrix('TableWithObjects2-1.asc'); 
ptCloud = pcdenoise(pointCloud(data)); 
pointData = ptCloud.Location; 

% Parameters
% Initial Distance & Angle Threshold
initialDistanceThreshold = 0.2;
initialAngleThreshold = 50; 
% Minimum Distance & Angle Threshold
minDistanceThreshold = 0.052; 
minAngleThreshold = 5; 
% Max times of iteration
maxIterations = 10; 
tolerance = 1e-3; 


currentDistanceThreshold = initialDistanceThreshold;
currentAngleThreshold = initialAngleThreshold;

for i = 1:maxIterations
    % PCA: Compute Dominate Plane
    [coeff, ~, ~, ~, ~, mu] = pca(pointData); 
    normal_vector = coeff(:, 3); 

    % Distance of each point to Dominate Plane
    point_cloud_centered = pointData - mu; 
    distances = abs(point_cloud_centered * normal_vector); 

    % Angle Difference of each point to Dominate Plane
    normals = pcnormals(pointCloud(pointData), 10);
    angleDifferences = acosd(dot(normals, repmat(normal_vector', size(normals, 1), 1), 2));

    % Filterout outliers
    inliers = pointData((distances < currentDistanceThreshold) & (angleDifferences < currentAngleThreshold), :);

    % Check Convergence
    if size(inliers, 1) / size(pointData, 1) > (1 - tolerance)
        break;
    end
    pointData = inliers;

    % Reduce Angle & Distance Threshold
    currentDistanceThreshold = max(currentDistanceThreshold * 0.8, minDistanceThreshold); 
    currentAngleThreshold = max(currentAngleThreshold * 0.8, minAngleThreshold); 
end


% Cluster remining inliers
epsilon = 0.3; 
minPts = 10;    
labels = dbscan(inliers, epsilon, minPts);
uniqueLabels = unique(labels);
% Table should be the clusters with the most dense points
maxClusterLabel = mode(labels(labels >= 0));
inliers = inliers(labels == maxClusterLabel, :);

% Plot point cloud
figure;
scatter3(inliers(:, 1), inliers(:, 2), inliers(:, 3), 10, inliers(:, 3), 'filled');
xlabel('X');
ylabel('Y');
zlabel('Z');
title(['Filtered Point Cloud with ' num2str(maxIterations) ' iterations']);
axis equal;
grid on;


% Compute Table inlier
[finalCoeff, ~, ~, ~, ~, finalMu] = pca(inliers);
planeNormal = finalCoeff(:, 3);
planePoint = finalMu; 
a = planeNormal(1);
b = planeNormal(2);
c = planeNormal(3);
d = -dot(planeNormal, planePoint);
fprintf('Table Parameter: %fx + %fy + %fz + %f = 0\n', a, b, c, d);
