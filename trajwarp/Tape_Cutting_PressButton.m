%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       Motion for Pressing Tool Button to Cut the Tape
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/08/25
%  Modified by Te Tang, 09/28/2016 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% There are some ad-hoc methods/settings here that may not be suitable for
% any general situations when the testbed has big structure changes. Better
% to improve with online collision free motion planning.

function Tape_Cutting_PressButton(si)

%% Initialization
ParamSgnID = si.ParamSgnID;
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(ParamSgnID, 'off');

% Here it is assumed that Robot 2 holds taping tool while Robot 1 is to
% push the cutting button to cut the tape.
opt.robot_idx = 1;
num = numel(ParamSgnID);

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

%% Stop the application
brake_on_off(ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);

end