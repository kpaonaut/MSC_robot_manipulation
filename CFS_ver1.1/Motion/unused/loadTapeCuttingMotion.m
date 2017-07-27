%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%   directly load robot joint data and save as a LTT data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function loadTapeCuttingMotion(filepath,filename)
load([filepath,filename])
% qVec: 4 Robot No.1 position
% rbt_pos.jnt: two robot init jnt pos
% tVec: TimeVec
N = size(qVec,1);
LTT_Data_PressTapingTool.DesJntPos{1} = qVec;
LTT_Data_PressTapingTool.DesJntPos{2} = repmat(rbt_pos.jnt(2, :),N,1);
LTT_Data_PressTapingTool.ReplayTime{1} = tVec;
LTT_Data_PressTapingTool.ReplayTime{2} = tVec;
LTT_Data_PressTapingTool.GrpCmd{1} = repmat([zeros(1,5), 1],N,1);
LTT_Data_PressTapingTool.GrpCmd{2} = repmat([zeros(1,5), 1],N,1);
LTT_Data_PressTapingTool.Marker_T_W = [];
save([filepath,'PressTapingTool'],'LTT_Data_PressTapingTool')

          