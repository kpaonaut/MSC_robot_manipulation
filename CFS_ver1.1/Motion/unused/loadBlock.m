%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%    load LTT data and save the block information
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function block = loadBlock(filepath, BlockName)

filepath = [filepath,'LTT_Data_',BlockName];
load(filepath);
varname = matlab.lang.makeValidName(['LTT_Data_',BlockName]); 
block.name = BlockName;
eval(['block.Marker_T_W = ',varname,'.Marker_T_W;']);