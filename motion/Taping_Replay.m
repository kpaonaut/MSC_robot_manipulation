%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       Main File for Taping Task Replay
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/08/25
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Taping_Replay(si)

ParamSgnID = si.ParamSgnID;
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(ParamSgnID, 'off');

% reset the gripper to fully open
gripper_action(ParamSgnID, 'DRIVE', cellstr(repmat('fully open', numel(ParamSgnID), 1)));  

% replay the teaching motion and circling and cutting motions
LTT_Motion_Planning(si);
Tape_Circling(si);
Tape_Cutting(si);

% Stop the application
brake_on_off(ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);
