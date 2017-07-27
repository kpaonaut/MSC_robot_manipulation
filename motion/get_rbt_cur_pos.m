%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       Get the current position for specific robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/08/22
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function rbt_pos = get_rbt_cur_pos(si, opt)

if nargin < 2,  opt = [];  end
if ~isfield(opt,'robot_idx') || isempty(opt.robot_idx),  opt.robot_idx = 1:si.robot_num;  end   % robot index set for which operations should be conducted (default: all robots)

rsi = si.ri(opt.robot_idx);
ParamSgnID = si.ParamSgnID(opt.robot_idx);
tg = SimulinkRealTime.target;
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(ParamSgnID, 'off');

rbt_pos.jnt = zeros(numel(opt.robot_idx), si.AxisNum);
for rn = 1:numel(opt.robot_idx)
    for i = 1:si.AxisNum
        rbt_pos.jnt(rn,i) = getsignal(tg, ParamSgnID{rn}.JntPos_deg(i));
    end
end
[rbt_pos.xyzwpr, rbt_pos.transM] = fanucfkine(rbt_pos.jnt, rsi{rn});

%% Stop the application
brake_on_off(ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);

end
