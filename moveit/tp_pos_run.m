%% Wenjie Chen, 2016/07/04, FANUC Corporation
% Shortcut of sequences to execute the specified position on Target PC

function tp_pos_run(si, PosCmd, opt)
% Input: PosCmd - rn * Axis_Num matrix

if nargin < 3,  opt = [];  end
if ~isfield(opt,'robot_idx') || isempty(opt.robot_idx),  opt.robot_idx = 1:si.robot_num;  end   % robot index set for which operations should be conducted (default: all robots)
if ~isfield(opt,'tcp') || isempty(opt.tcp),  opt.tcp = 0;  end      % 0 - joint position (default); 1 - tcp position
if ~isfield(opt,'SI_jnt') || isempty(opt.SI_jnt),  opt.SI_jnt = 0;  end   % 0 - degree (default); 1 - radian
if ~isfield(opt,'SI_tcp') || isempty(opt.SI_tcp),  opt.SI_tcp = 0;  end   % 0 - degree, mm (default); 1 - radian, m

if (size(PosCmd,2) ~= si.AxisNum && ~opt.tcp) || (size(PosCmd,2) ~= 6 && opt.tcp)
    PosCmd = PosCmd';  
end
if numel(opt.robot_idx) ~= size(PosCmd,1)
    error('Dimensions of position commands are not the same as robot number!');
end
rsi = si.ri(opt.robot_idx);     ParamSgnID = si.ParamSgnID(opt.robot_idx);

disp('---> Now starting to move robots to desired positions ...');
tg = SimulinkRealTime.target;
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(ParamSgnID, 'off');

if ~isempty(PosCmd)         % set the new position command
    if opt.tcp
        for rn = 1:numel(opt.robot_idx)
            last_jnt = getsignal(tg, ParamSgnID{rn}.JntPos_deg);
            if opt.SI_jnt,  last_jnt = deg2rad(last_jnt);  end;
            PosCmd(rn,:) = fanucikine(PosCmd(rn,:), rsi{rn}, last_jnt, opt);
        end
    end
    
    if opt.SI_jnt,  PosCmd = rad2deg(PosCmd);  end;     % if it is radian, need to convert to degree
    for rn = 1:numel(opt.robot_idx)
        for i = 1:si.AxisNum
            setparam(tg, ParamSgnID{rn}.IniJntPos(i), PosCmd(rn,i));
        end
    end
    pause(0.1);     % wait for the mode controller to take in this input
end

for rn = 1:numel(opt.robot_idx)
    setparam(tg, ParamSgnID{rn}.Tp_Pos_Run, 1);
end

wait_finish_motion(ParamSgnID);

% deactivate the tp_pos_run
for rn = 1:numel(opt.robot_idx)
    setparam(tg, ParamSgnID{rn}.Tp_Pos_Run, 0);
end

brake_on_off(ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);

fprintf('---> Robots arrived at the desired positions.\n\n');

end
