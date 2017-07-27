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

function Tape_Cutting_Retreat(si)

%% Initialization
ParamSgnID = si.ParamSgnID;
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(ParamSgnID, 'off');

% Here it is assumed that Robot 2 holds taping tool while Robot 1 is to
% push the cutting button to cut the tape.
opt.robot_idx = 1;

%% Move Robot 1 from previous circling position to cutting button position
% Disengage Robot 1 from the cable and move to more spacious position
%gripper_action(ParamSgnID(opt.robot_idx), 'DRIVE', {'center position'});
offset = [0, 0, -120, 0, 0, 0];      % move robot 1 along tool frame Z axis, -50mm, to disengage with the cable
relative_move(si, offset, [], opt);
gripper_action(ParamSgnID(opt.robot_idx), 'DRIVE', {'fully closed'});
%offset = [0, 250, 100, 0, 0, 0];    % move robot 1 along world frame Z axis, 100mm, Y axis 250mm, to get more free space
offset = [0, 150, 70, 0, 0, 0];    % move robot 1 along world frame Z axis, 100mm, Y axis 250mm, to get more free space
opt.frame = 'world';
relative_move(si, offset, [], opt);

%% Stop the application
brake_on_off(ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);

end