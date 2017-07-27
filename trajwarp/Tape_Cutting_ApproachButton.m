%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       Main File for Cutting the Tape
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/08/25
%  Modified by Te Tang, 09/28/2016 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% There are some ad-hoc methods/settings here that may not be suitable for
% any general situations when the testbed has big structure changes. Better
% to improve with online collision free motion planning.

function [qVec, tVec, rbt_pos] = Tape_Cutting_ApproachButton(si)

%% Initialization
ParamSgnID = si.ParamSgnID;
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(ParamSgnID, 'off');

% Here it is assumed that Robot 2 holds taping tool while Robot 1 is to
% push the cutting button to cut the tape.
opt.robot_idx = 1;
rsi = si.ri{opt.robot_idx};

%% Move Robot 1 to the cutting button position with good orientation for pushing button
rbt_pos = get_rbt_cur_pos(si);
taping_tool = get_taping_tool_pos(si);
% add J6 = 0 as an intermediate step to avoid bad posture/collision (this
% is not a clever method, better to do online collision-free motion planning)
midPoint1 = [rbt_pos.jnt(1,1:5), 0];
midPoint2 = fanucikine(taping_tool.cut_btn_B_r_off, rsi, midPoint1);
finalPoint = fanucikine(taping_tool.cut_btn_B_r, rsi, midPoint2);
qVec = [rbt_pos.jnt(1,:); midPoint1; midPoint2; finalPoint];
tVec = [3, 6, 2];  

%% Stop the application
brake_on_off(ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);

end