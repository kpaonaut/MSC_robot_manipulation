%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%    load LTT data from a Example file and save motion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function motion = loadMotionExample(filepath, ActionName,robot_num)

load(filepath);

motion.ActionName = ActionName;
varname = matlab.lang.makeValidName(['LTT_Data_',ActionName]); 
eval(['motion.DesJntPos = ',varname,'.DesJntPos{robot_num};']);
eval(['motion.ReplayTime = ',varname,'.ReplayTime{robot_num};']);
eval(['motion.GrpCmd = ',varname,'.GrpCmd{robot_num};']);
eval(['motion.Marker_T_W = ',varname,'.Marker_T_W;']);
ReplayTimeSize = size(motion.ReplayTime,1);

motion.MoFlag = zeros(ReplayTimeSize,1);
% motion{2}.MoFlag = zeros(size(ReplayTime,1),1);

for r = 1:1
    for i = 1:ReplayTimeSize
        delta = norm( motion.DesJntPos(i+1,:) - motion.DesJntPos(i,:) );
        if delta >1, motion.MoFlag(i) = 1; end 
    end
end

