%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Warp LTT_Data by TPS function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function New_LTT_Data = LTT_Warping(Old_LTT_Data, NewFeaturePoints, NewMarker_T_W, si, OldFeaturePoints, robot_idx)

% specify old feature points
if nargin < 5;
    OldFeaturePoints = Old_LTT_Data.FeaturePoint_xyz_W;
end

% specify which arm to warp
if nargin < 6
    robot_idx = [1,2];
end
    
% generate warp function handle
lambda = 0.01; % TPS regularization coefficient
[A, B, c] = compute_warp(OldFeaturePoints, NewFeaturePoints, lambda);   
warp = make_warp(A, B, c, OldFeaturePoints);

% create New_LTT_Data after warping
New_LTT_Data = Old_LTT_Data;
New_LTT_Data.Marker_T_W = NewMarker_T_W;
New_LTT_Data.FeaturePoint_xyz_W = NewFeaturePoints;
for i = robot_idx
    for j = 1:size(Old_LTT_Data.TCP_T_W{i},3)
        New_LTT_Data.TCP_T_W{i}(:,:,j) = TPS_warp_T(Old_LTT_Data.TCP_T_W{i}(:,:,j), warp);
        New_LTT_Data.TCP_xyzwpr_W{i}(j,:) = T2xyzwpr(New_LTT_Data.TCP_T_W{i}(:,:,j));
        New_LTT_Data.TCP_T_B{i}(:,:,j) = FrameTransform(New_LTT_Data.TCP_T_W{i}(:,:,j), 'T', 'W2B', si, i);
        New_LTT_Data.TCP_xyzwpr_B{i}(j,:) = T2xyzwpr(New_LTT_Data.TCP_T_B{i}(:,:,j));
        % Joint Position
        New_LTT_Data.DesJntPos{i}(j,:) = fanucikine(New_LTT_Data.TCP_xyzwpr_B{i}(j,:), si.ri{i}, Old_LTT_Data.DesJntPos{i}(j,:));
    end
end
