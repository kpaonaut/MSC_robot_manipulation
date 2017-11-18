for i = 3:3
load(['F:\WANGRUI\MSC_robot_manipulation-master\MSC_robot_manipulation-master\WangRui\data\Knot\trainingRope_',num2str(i),'.mat']); 
% load the training data of trajectory transform points from kinect frame to world frame
points_U = points_U(:, 1:3);
points_W{i} = transformU2W(points_U, si); % training rope transformed
points_W{i} = points_W{i}(:, 1:2);
plot(points_W{1}(:, 1), points_W{1}(:, 2));hold on;scatter(points_W{1}(1, 1), points_W{1}(1, 2))
scatter(points_W{3}(:, 1), points_W{3}(:, 2));hold on;scatter(points_W{3}(1, 1), points_W{3}(1, 2))
end