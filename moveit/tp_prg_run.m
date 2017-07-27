%% Wenjie Chen, 2016/07/04, FANUC Corporation
% Shortcut of sequences to execute the external motion commands on Target PC

function tp_prg_run(si, opt)

if nargin < 2,  opt = [];  end
if ~isfield(opt,'robot_idx') || isempty(opt.robot_idx),  opt.robot_idx = 1:si.robot_num;  end   % robot index set for which operations should be conducted (default: all robots)
ParamSgnID = si.ParamSgnID(opt.robot_idx);

disp('---> Now starting to execute planned robot motions ...');
tg = SimulinkRealTime.target;
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(ParamSgnID, 'off');

% activate the tp_prg_run
for rn = 1:numel(opt.robot_idx)
    setparam(tg, ParamSgnID{rn}.Tp_Prg_Run, 1);
end

wait_finish_motion(ParamSgnID);

% deactivate the tp_prg_run
for rn = 1:numel(opt.robot_idx)
    setparam(tg, ParamSgnID{rn}.Tp_Prg_Run, 0);
end

brake_on_off(ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);

fprintf('---> Execution of the planned motions is finished.\n\n');

end
