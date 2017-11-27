%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Warp LTT_Data by CPD function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Rui Wang, 11/26/2017       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function LTT_Data_Test = warpFn(LTT_Data_Train, idx, j, warp, si)
%% create New_LTT_Data after warping
LTT_Data_Test = LTT_Data_Train;
i = idx;
LTT_Data_Test.TCP_T_W{i}(:,:,j) = TPS_warp_T_2D(LTT_Data_Train.TCP_T_W{i}(:,:,j), warp);
LTT_Data_Test.TCP_xyzwpr_W{i}(j,:) = T2xyzwpr(LTT_Data_Test.TCP_T_W{i}(:,:,j));
LTT_Data_Test.TCP_T_B{i}(:,:,j) = FrameTransform(LTT_Data_Test.TCP_T_W{i}(:,:,j), 'T', 'W2B', si, i);
LTT_Data_Test.TCP_xyzwpr_B{i}(j,:) = T2xyzwpr(LTT_Data_Test.TCP_T_B{i}(:,:,j));
% Joint Position
LTT_Data_Test.DesJntPos{i}(j,:) = fanucikine(LTT_Data_Test.TCP_xyzwpr_B{i}(j,:), si.ri{i}, LTT_Data_Train.DesJntPos{i}(j,:));