%% Wenjie Chen, 2016/07/27, FANUC Corporation
% This is to move robots back to their initial positions.

function init_pos_back(si, opt)

if nargin < 2,  opt = [];  end
if ~isfield(opt,'robot_idx') || isempty(opt.robot_idx),  opt.robot_idx = 1:si.robot_num;  end   % robot index set for which operations should be conducted (default: all robots)

% Set the robot initial position
rsi = si.ri(opt.robot_idx);
PosCmd = zeros(numel(opt.robot_idx), si.AxisNum);

for rn = 1 : numel(opt.robot_idx)
    PosCmd(rn, :) = rsi{rn}.iniJntPos;
end

tp_pos_run(si, PosCmd, opt);

end
