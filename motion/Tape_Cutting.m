%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       Main File for Cutting the Tape
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/08/25
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% There are some ad-hoc methods/settings here that may not be suitable for
% any general situations when the testbed has big structure changes. Better
% to improve with online collision free motion planning.

function Tape_Cutting(si)

%% Initialization
ParamSgnID = si.ParamSgnID;
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(ParamSgnID, 'off');

% Here it is assumed that Robot 2 holds taping tool while Robot 1 is to
% push the cutting button to cut the tape.
opt.robot_idx = 1;
rsi = si.ri{opt.robot_idx};
num = numel(ParamSgnID);

%% Move Robot 1 from previous circling position to cutting button position
% Disengage Robot 1 from the cable and move to more spacious position
gripper_action(ParamSgnID(opt.robot_idx), 'DRIVE', {'center position'});
offset = [0, 0, -120, 0, 0, 0];      % move robot 1 along tool frame Z axis, -50mm, to disengage with the cable
relative_move(si, offset, [], opt);
gripper_action(ParamSgnID(opt.robot_idx), 'DRIVE', {'fully closed'});
%offset = [0, 250, 100, 0, 0, 0];    % move robot 1 along world frame Z axis, 100mm, Y axis 250mm, to get more free space
offset = [0, 150, 70, 0, 0, 0];    % move robot 1 along world frame Z axis, 100mm, Y axis 250mm, to get more free space
opt.frame = 'world';
relative_move(si, offset, [], opt);

% Move Robot 1 to the cutting button position with good orientation for pushing button
rbt_pos = get_rbt_cur_pos(si, opt);
taping_tool = get_taping_tool_pos(si);
% add J6 = 0 as an intermediate step to avoid bad posture/collision (this
% is not a clever method, better to do online collision-free motion planning)
midPoint1 = [rbt_pos.jnt(1:5), 0];
midPoint2 = fanucikine(taping_tool.cut_btn_B_r_off, rsi, midPoint1);
finalPoint = fanucikine(taping_tool.cut_btn_B_r, rsi, midPoint2);
qVec = [rbt_pos.jnt; midPoint1; midPoint2; finalPoint];
tVec = [3, 6, 2];   
opt.interpType = 'tcp';
%opt.interpType = 'joint';
extcmd = ExtCmdGenerate(qVec, tVec, si, rsi, [], opt);
ExtCmdDownload(ParamSgnID{opt.robot_idx}, extcmd);
tp_prg_run(si, opt);

%% Cut the tape with Gripper 1 & 2 motions
% Move Gripper 1 along tool frame to push taping tool button
push_button_offset = [-10, -25, 0];
opt.robot_idx = 1;
offset = zeros(num, 6);
offset(opt.robot_idx, 1:3) = push_button_offset;      
relative_move(si, offset, []);

% Move Gripper 1 & 2 together along world frame to cut the tape
cut_tape_offset = ones(2,2) * 20;
offset = zeros(num, 6);
offset(:,[1,3]) = cut_tape_offset;    
opt.robot_idx = 1:num;
opt.frame = 'world';
relative_move(si, offset, [], opt);

pause(1);

% Move Gripper 1 & 2 together along world frame to previous location
offset = zeros(num, 6);
offset(:,[1,3]) = -cut_tape_offset;    
opt.robot_idx = 1:num;
opt.frame = 'world';
relative_move(si, offset, [], opt);

% Move Gripper 1 along tool frame to the last position before pushing button
opt.robot_idx = 1;
offset = zeros(num, 6);
offset(opt.robot_idx, 1:3) = -push_button_offset;      
relative_move(si, offset, []);

%% Move Robot 1 & 2 away
% Move Robot 1 away from Robot 2 to more spacious location
opt.robot_idx = 1;
rbt_pos = get_rbt_cur_pos(si, opt);
qVec = [rbt_pos.jnt; midPoint2; midPoint1];
tVec = [2, 6];   
opt.interpType = 'tcp';
extcmd = ExtCmdGenerate(qVec, tVec, si, rsi, [], opt);
ExtCmdDownload(ParamSgnID{opt.robot_idx}, extcmd);
tp_prg_run(si, opt);

% Reverse the Robot 2 teaching motion to put the taping tool back to stand
% opt.robot_idx = 2;
% opt.reverse = 1;
% LTT_Motion_Planning(si, opt);

%% Stop the application
brake_on_off(ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);

end