function [pcloud, distance] = depthToCloud(depth, topleft)
% depthToCloud.m - Convert depth image into 3D point cloud
% Author: Liefeng Bo and Kevin Lai
%
% Input: 
% depth - the depth image
% topleft - the position of the top-left corner of depth in the original depth image. Assumes depth is uncropped if this is not provided
%
% Output:
% pcloud - the point cloud, where each channel is the x, y, and z euclidean coordinates respectively. Missing values are NaN.
% distance - euclidean distance from the sensor to each point
%
% Modified by:  Wenjie Chen, FANUC Corporation, 2015/12/18
% Modified by: Te Tang, 2017/4/16    change focalLength(constant)
% Currently written for Kinect V2 only

if nargin < 2
    topleft = [1 1];
end

depth= double(depth);
depth(depth == 0) = nan;

% RGB-D camera constants
center = [960+0.5, 540+0.5];  %[960-0.5 540-0.5];
[imh, imw] = size(depth);
focalLength = mean([1090.96136630252,1080.92855651613]); %[1059.90324304455,1053.38419482443]); % 1081.37; 
MM_PER_M = 1; % use mm as final unit; 1000;

% convert depth image to 3d point clouds
pcloud = zeros(imh,imw,3);
xgrid = ones(imh,1)*(1:imw) + (topleft(1)-1) - center(1);
ygrid = (1:imh)'*ones(1,imw) + (topleft(2)-1) - center(2);
pcloud(:,:,1) = xgrid.*depth/focalLength/MM_PER_M;  % flip the x axis
pcloud(:,:,2) = -ygrid.*depth/focalLength/MM_PER_M;  % flip the y axis. add - to counter flip
pcloud(:,:,3) = depth/MM_PER_M;
distance = sqrt(sum(pcloud.^2,3));
