% Wenjie Chen, FANUC Corporation, 2016/02/01
% Project 3D point clouds to a best fitted 2D plane
% Output projected 2D point clouds

function [pt2d, model] = proj3Dto2D(pt3d)

if ~isa(pt3d, 'pointCloud')
    pt3d = pointCloud(pt3d);
end

maxDistance = 1;
model = pcfitplane(pt3d, maxDistance);
pt2d = pt3d.Location * null(model.Normal);

% figure; pcshow(pt3d);
% figure; plot(pt2d(:,1), pt2d(:,2), 'o'); axis equal;

end