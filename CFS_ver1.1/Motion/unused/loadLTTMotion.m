%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%         load LTT data and save as a motion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function motion = loadLTTMotion(filepath, ActionName, robot_num)
filepath = [filepath,'LTT_Data_',ActionName];
load(filepath);

motion.ActionName = ActionName;
varname = matlab.lang.makeValidName(['LTT_Data_',ActionName]); 
eval(['motion.DesJntPos = ',varname,'.DesJntPos{robot_num};']);
eval(['motion.ReplayTime = ',varname,'.ReplayTime{robot_num};']);
eval(['motion.GrpCmd = ',varname,'.GrpCmd{robot_num};']);
eval(['motion.Marker_T_W = ',varname,'.Marker_T_W;']);
ReplayTimeSize = length(motion.ReplayTime);

motion.MoFlag = zeros(ReplayTimeSize,1);


for r = 1:1
    for i = 1:ReplayTimeSize
        delta = norm( motion.DesJntPos(i+1,:) - motion.DesJntPos(i,:) );
        if delta >1, motion.MoFlag(i) = 1; end 
    end
end

