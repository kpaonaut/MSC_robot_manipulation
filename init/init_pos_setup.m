%% Wenjie Chen, 2016/06/24, FANUC Corporation

% This is to set robot dependent parameters after model building and 
% move robots and grippers to their initial positions. The
% parameters set by model arguments are not tunable parameters during run
% time. Thus the desired runtime tunable parameters have to be initilized
% using global variable and then tuned specifically for each robot after
% model building.

function init_pos_setup(si, opt)

if nargin < 2,  opt = [];  end
if ~isfield(opt,'robot_idx') || isempty(opt.robot_idx),  opt.robot_idx = 1:si.robot_num;  end   % robot index set for which operations should be conducted (default: all robots)
if ~isfield(opt,'init_Move_On') || isempty(opt.init_Move_On),  opt.init_Move_On = 1;  end       % 1: with initilization move (default);  0: no init move

rsi = si.ri(opt.robot_idx);     ParamSgnID = si.ParamSgnID(opt.robot_idx);
tg = SimulinkRealTime.target;

%% Set the robot initial position
for rn = 1 : numel(opt.robot_idx)
    for i = 1:si.AxisNum
        setparam(tg, ParamSgnID{rn}.IniJntPos(i), rsi{rn}.iniJntPos(i));
        setparam(tg, ParamSgnID{rn}.IniTcpPos(i), rsi{rn}.iniTcpPos(i));
    end   
end

%% Generate initial external motion trajectory and download to target PC
for rn = 1 : numel(opt.robot_idx)
    disp(['---> Now generating motion commands for Robot No. ', num2str(rsi{rn}.robot_no), '...']);
    
    qVec = [rsi{rn}.iniJntPos;...
        rsi{rn}.iniJntPos + [rsi{rn}.iniJ1Off, -15, 15, 30, 30, 30];...
        rsi{rn}.iniJntPos];
    tVec = ones(size(qVec,1)-1,1) * 5;    % use 5 sec to move to the next pos
    
    % Generate external command and download to target PC
    extcmd = ExtCmdGenerate(qVec, tVec, si, rsi{rn});
    ExtCmdDownload(ParamSgnID{rn}, extcmd);
end

pause(0.1);     % wait for the mode controller to take in this input
fprintf('---> Tunable parameter values are initialized.\n\n');

%% Go to initial positions and set up grippers
if opt.init_Move_On
    wasStopped = tg_start_stop('start');    

    % robots execute initial external commands
    tp_prg_run(si, opt);   

    % setup and initialize grippers
    gripper_action(ParamSgnID, 'RESET');
    gripper_action(ParamSgnID, 'SVON');
    gripper_action(ParamSgnID, 'SETUP');
    gripper_action(ParamSgnID, 'DRIVE', cellstr(repmat('fully open', numel(opt.robot_idx), 1)));   
    
    fprintf('\n---> The robots and the grippers are reset at their initial positions.\n\n');
    
    tg_start_stop('stop', wasStopped);
end

%% System ID for F/T sensor and payload
if si.FS_ID_On  
   FT_sensor_identification(si, opt);
end

end
