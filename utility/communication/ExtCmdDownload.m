%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor  
%       Download motion command from Host PC to Target PC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created on 06/24/2016 by Wenjie Chen, FANUC Corporation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ExtCmdDownload(ParamSgnID, extcmd)

%% Create target object
tg = SimulinkRealTime.target;

%% Transfer Robot No.1 motion cmd to Target PC
%Set value to parameter represented by paramid 
setparam(tg, ParamSgnID.const_ExtCmd_vcmd, extcmd.vcmd_t);
setparam(tg, ParamSgnID.const_ExtCmd_vcmd_EndCnt, extcmd.vcmd_EndCnt_t);
setparam(tg, ParamSgnID.const_ExtCmd_inipos, extcmd.inipos_t);
setparam(tg, ParamSgnID.const_ExtCmd_grippercmd, extcmd.grippercmd);

%%
fprintf(['-----> Motion Command Download to Robot No. ', num2str(ParamSgnID.robot_no),  ' has completed.\n\n']);
