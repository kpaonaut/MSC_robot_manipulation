%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       Get the position specs for taping tool
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/08/22
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function taping_tool = get_taping_tool_pos(si, opt)

if nargin < 2,  opt = [];  end
if ~isfield(opt,'robot_idx') || isempty(opt.robot_idx),  opt.robot_idx = si.robot_num;  end   % robot index set which holds taping tool (default: last robot)

rsi = si.ri(opt.robot_idx);
taping_tool.rbt_pos = get_rbt_cur_pos(si, opt);     % get the robot position which holds taping tool

taping_tool.radius = 55/1000;    % radius of the taping circle, m
taping_tool.center_T = [0, 0, 90]/1000;    % taping circle center at tool frame, m
taping_tool.center_B = taping_tool.rbt_pos.transM * [taping_tool.center_T'; 1];   % taping circle center at base frame
taping_tool.center_B = taping_tool.center_B(1:3)';
taping_tool.open_pt_T = taping_tool.center_T + [15, 0, 70]/1000;  % the open point of the taping tool in tool frame, m
taping_tool.open_pt_B = taping_tool.rbt_pos.transM * [taping_tool.open_pt_T'; 1];   % the open point of the taping tool in base frame
taping_tool.open_pt_B = taping_tool.open_pt_B(1:3)';

finger_width = 35;          % the side width of the finger from tcp to top of rubber, mm
rot_angle = [90, 50, 180];  % the relative rotation angle (degree) from this robot tool frame to the other robot tool frame that can press the cutting button
taping_tool.cut_btn_T = [taping_tool.center_T*1000 + [-100-finger_width*sind(rot_angle(2)), 20, finger_width*cosd(rot_angle(2))], rot_angle];  % the cut button of the taping tool in tool frame, xyzwpr
taping_tool.cut_btn_B = taping_tool.rbt_pos.transM * xyzwpr2T(taping_tool.cut_btn_T);   % the cut button of the taping tool in base frame
taping_tool.cut_btn_W = rsi{1}.T_B2W * taping_tool.cut_btn_B;   % the cut button of the taping tool in base frame
taping_tool.cut_btn_W_off = taping_tool.cut_btn_W;
taping_tool.cut_btn_W_off(1:2, 4) = taping_tool.cut_btn_W_off(1:2, 4) + [-0.05; 0.1];    % get an intermediate point with some offsets

opt.robot_idx = 3 - opt.robot_idx;  % get the other robot index
rsi = si.ri(opt.robot_idx);
taping_tool.cut_btn_B_r = rsi{1}.T_B2W \ taping_tool.cut_btn_W;   % the cut button of the taping tool in base frame of the other robot
taping_tool.cut_btn_B_r_off = rsi{1}.T_B2W \ taping_tool.cut_btn_W_off;   

end
