%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       Load the LTT data from Target PC and save into /data folder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, UC Berkeley, 09/28/2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% load data after LTT teaching
function LTT_Data = Load_LTT(si)
opt = {};
opt.is_dual_arm = true;
opt.robot_idx = [1,2];
opt.DataMode = 1;     % 0 - use previous LTT data to replay (default); 1 - get new LTT data but no replay
LTT_Data = Load_LTT_Data(si, opt);