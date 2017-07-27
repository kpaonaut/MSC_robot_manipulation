%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       Main File for Tape Circling Task
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/07/20
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Tape_Circling(si, opt)

if nargin < 2,  opt = [];  end
if ~isfield(opt,'robot_idx') || isempty(opt.robot_idx),  opt.robot_idx = si.robot_num;  end   % robot index set which holds taping tool (default: last robot)

rsi = si.ri{opt.robot_idx};
ParamSgnID = si.ParamSgnID{opt.robot_idx};
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(ParamSgnID, 'off');

%% Compute the circle trajectory
num_round = 2;
taping_tool = get_taping_tool_pos(si, opt);

theta = (360*num_round-90:-5:-68)'/180*pi; % the angle of circling (num of rounds - some offset)
taping_circle_W = [cos(theta), zeros(size(theta)), sin(theta)] * taping_tool.radius;    % the taping circle position in world frame
taping_circle_B = (rsi.T_B2W(1:3,1:3)' * taping_circle_W')' + repmat(taping_tool.center_B, numel(theta), 1);   % the taping circle position in base frame
offset_circle_B = repmat(taping_tool.open_pt_B, numel(theta), 1) - taping_circle_B;    % the offset from taping tool open point to the desired taping circle position
offset_circle_B = [zeros(1,3); offset_circle_B];    % add first step to start from the current initial position
%ending_offset_T = [-5, 1, -43]/1000;    % final ending offset in tool frame to move the tool away from cable, m
ending_offset_T = [-5, 8.5, -43]/1000;    % final ending offset in tool frame to move the tool away from cable, m
ending_offset_B = (taping_tool.rbt_pos.transM(1:3,1:3) * ending_offset_T')';  % final ending offset in base frame
tcp_circle_B = [offset_circle_B*1000 + repmat(taping_tool.rbt_pos.xyzwpr(1:3), size(offset_circle_B,1), 1), ...
    repmat(taping_tool.rbt_pos.xyzwpr(4:6), size(offset_circle_B,1), 1)];  % desired tcp position to accomplish taping circle
tcp_circle_B(end+1,:) = tcp_circle_B(end,:) + [ending_offset_B*1000, 0, 0, 0];   % add the ending offset

qVec = fanucikine(tcp_circle_B, rsi, taping_tool.rbt_pos.jnt);
tVec = ones(size(qVec,1)-1, 1) * 0.1;   % use 0.1 sec for every 5 degree motion
tVec([1, end]) = 0.5;   % use 0.5 sec for the final ending offset

%% Generate external command and download to target PC
opt.interpType = 'linear';
extcmd = ExtCmdGenerate(qVec, tVec, si, rsi, [], opt);
ExtCmdDownload(ParamSgnID, extcmd);

%% Execute the trajectory on target PC
tp_prg_run(si, opt);

%% Stop the application
brake_on_off(ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);

end
