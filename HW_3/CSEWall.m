data = readmatrix('CSE-1.asc');
ptCloud = pcdenoise(pointCloud(data));
pointData = ptCloud.Location;

% Find Floor
% PCA: Compute Floor Plane
[coeff, ~, ~, ~, ~, mu] = pca(pointData);
normal_vector = coeff(:, 3);
maxDistance = 0.5;
referenceVector = normal_vector;
maxAngularDistance = 5;
% Plane Fitting: Floor Plane
[model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,...
    maxDistance,referenceVector,maxAngularDistance);
plane1 = select(ptCloud,inlierIndices);
remainPtCloud = select(ptCloud,outlierIndices);

% Find First Wall
% PCA: Compute First Wall
remaining_points = remainPtCloud.Location;
[wall_coeff, ~, ~, ~, ~, ~] = pca(remaining_points);
% First Wall Normal: Perpendicular to floor
angles = zeros(3,1);
floor_normal = normal_vector; 
for i = 1:3
    angles(i) = abs(dot(wall_coeff(:,i), floor_normal));
end
[~, idx] = min(angles);
wall_reference = wall_coeff(:, idx);
wall_reference = wall_reference - (dot(wall_reference, floor_normal) * floor_normal);
wall_reference = wall_reference / norm(wall_reference);
% Plane Fitting: First Wall
maxDistance = 0.6;
maxAngularDistance = 10;
[wallModel, wallInliers, wallOutliers] = pcfitplane(remainPtCloud, ...
    maxDistance, wall_reference, maxAngularDistance);
wall1 = select(remainPtCloud, wallInliers);

% Find Remaining Wall and Objects
remaining_points = select(remainPtCloud, wallOutliers);
wall_planes = {}; 
minPoints = 500;
wall_plane_param = {};
for iter = 1:5  
    maxDistance = 0.5;
    maxAngularDistance = 10;
    % Wall & Object Normal: Perpendicular to First Wall and Floor
    referenceVector = cross(floor_normal, first_wall_normal);
    referenceVector = referenceVector / norm(referenceVector);
    % Plane Fitting
    [model, inlierIndices, outlierIndices] = pcfitplane(remaining_points, ...
        maxDistance, referenceVector, maxAngularDistance);
    if numel(inlierIndices) < minPoints
        break;
    end
    new_wall = select(remaining_points, inlierIndices);
    wall_planes{end+1} = new_wall; 
    wall_plane_param{end+1} = model;

    % Remove found Wall & Object From the cloud
    validOutliers = outlierIndices(outlierIndices <= remaining_points.Count);
    remaining_points = select(remaining_points, validOutliers);
end
% Visualize
figure;
% Floor
pcshow(plane1.Location, 'g'); 
hold on;
% First Wall
pcshow(wall1.Location, 'r');
fprintf('Parameters of red wall:\n');
disp(wallModel);
% Each other Objects & Wall will be assigned with random color
for i = 1:length(wall_planes)
    pcshow(wall_planes{i}.Location, rand(1, 3));
    wall_center = mean(wall_planes{i}.Location, 1);
    text(wall_center(1), wall_center(2), wall_center(3), sprintf('Wall %d', i), ...
         'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');  
    fprintf('Parameters of wall %d:\n', i);
    disp(wall_plane_param{i});  
end
% Remaining Point Will be Black
pcshow(remaining_points.Location, 'k');
title('Ground and Walls');
xlabel('X');
ylabel('Y');
zlabel('Z');
legend('Floor', 'Wall 1', 'Additional Walls', 'Remaining Points');
