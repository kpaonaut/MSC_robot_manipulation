%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%    load LTT data and save the block properties
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%  This function is loading the "Marker_T_W" and "DesJntPos" to the block.
%  Use "buildBlkObs" to generate other block properties.
% 
%  Block Properties:
%  block.name       : the name of the block
%  block.Marker_T_W : the marker's transformation matrix
%  block.DesJntPos  : the robot's position related to this motion (only valid for PressTapingTool now)
%  block.type       : the shape definition, which determines the distance calculation method
%  block.p          : the characteristic points of the block
%  block.o          : the origin of the block CAD model (for visualization)
%  block.r          : the radius of the block
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
function block = loadBlock2(filepath, filename, BlockName)

filepath = [filepath, filename];
load(filepath);
varname = matlab.lang.makeValidName(['LTT_Data_',BlockName]); 
block.name = BlockName;
eval(['block.Marker_T_W = ',varname,'.Marker_T_W;']);
block.Marker_T_W(3,4) = block.Marker_T_W(3,4) + 10; % add-hoc 
eval(['block.DesJntPos = ',varname,'.DesJntPos;']);