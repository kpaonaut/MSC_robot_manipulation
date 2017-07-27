%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%      Generate lead through teaching motion command to Target PC from Record data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/07/01
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [DesJntPos, ReplayTime, GrpCmd] = LTT_Motion_Planning(si, opt)

if nargin < 2,  opt = [];  end
if ~isfield(opt,'robot_idx') || isempty(opt.robot_idx),  opt.robot_idx = 1:si.robot_num;  end   % robot index set for which operations should be conducted (default: all robots)
if ~isfield(opt,'relPath2Data') || isempty(opt.relPath2Data),  opt.relPath2Data = 'data\';  end
if ~isfield(opt,'DataMode') || isempty(opt.DataMode),  opt.DataMode = 0;  end   % 0 - use previous LTT data to replay (default); 1 - get new LTT data but no replay
if ~isfield(opt,'speedup') || isempty(opt.speedup),  opt.speedup = 2.5;  end   % Define the robot speed relative to LTT
if ~isfield(opt,'fname_prefix') || isempty(opt.fname_prefix),  opt.fname_prefix = ['LTT_Data_'];  end   % Define the filename prefix for LTT record data file
if ~isfield(opt,'LTT') || isempty(opt.LTT),  opt.LTT = 1;  end   % 0 - not in LTT mode; 1 - in LTT mode (default)
if ~isfield(opt,'interpType') || isempty(opt.interpType),  opt.interpType = 'tcp';  end   % interpolation at: 'joint' space; 'TCP' space (default); 'no' interpolation
if ~isfield(opt,'reverse') || isempty(opt.reverse),  opt.reverse = 0;  end   % 0: no reverse (default); 1: reverse the motion from end to beginning

rsi = si.ri(opt.robot_idx);     ParamSgnID = si.ParamSgnID(opt.robot_idx);

% Initialization for load file name
loadfile = cell(size(opt.robot_idx));
for rn = 1:numel(opt.robot_idx)
    loadfile{rn} = ['LTTData', num2str(rsi{rn}.robot_no)];
end

%% Load Recorded Position from Target PC
if opt.DataMode == 1
    disp('---> Saving Lead Through Teaching Data.')
    LTTData = DataLog(loadfile, opt);
else
    fList = dir([opt.relPath2Data, opt.fname_prefix, '*.mat']);
    LTTData = load([opt.relPath2Data, fList(end).name]);
end

%% Generate Replay Motion Command
if opt.DataMode,  return;  end   % if only get new data and no replay, then finish

disp('---> Generating robot motion trajectory.')

Data = cell(numel(rsi),1);          ind = cell(numel(rsi),1);  
IniJntPos = zeros(numel(rsi),si.AxisNum);     str_time = zeros(numel(rsi),1);
valid_idx = [];
for rn = 1:numel(rsi)
    Data{rn} = LTTData.(loadfile{rn});
    ind{rn} = find(Data{rn}(:,1)~=0);       % Get the recorded data index (when J1 angle is not 0)
    if numel(ind{rn}) > 1,  valid_idx(end+1) = rn;  end
end

% if there is robot not moving at all, update robot_idx and rerun this function
if numel(valid_idx) == 0
    fprintf('No robot motion recorded and replayed. \n\n');
    return;
end
if numel(valid_idx) ~= numel(rsi)
    opt.robot_idx = opt.robot_idx(valid_idx);
    LTT_Motion_Planning(si, opt);
    return;
end

% reverse the motion command sequence if desired
if opt.reverse
    for rn = 1:numel(rsi)
        Data{rn}(:,end) = Data{rn}(end,end) - Data{rn}(:,end);   % update the time stamp
        ind{rn} = size(Data{rn},1) + 1 - ind{rn};   % update the valid index
        Data{rn} = Data{rn}(end:-1:1, :);   % reverse the sequence
        ind{rn} = ind{rn}(end:-1:1);   % reverse the sequence
    end    
end

for rn = 1:numel(rsi)
    str_time(rn) = ind{rn}(1) * diff(Data{rn}(1:2,end));     % Get the starting time, sec
end
str_time = str_time - min(str_time);

for rn = 1:numel(rsi)
    DesJntPos = Data{rn}(ind{rn},1:si.AxisNum);      % jntpos, deg   
    IniJntPos(rn,:) = DesJntPos(1,:);
    LTTtime = diff(Data{rn}(ind{rn},end));   % sec
    GrpCmd = Data{rn}(ind{rn},si.AxisNum+1:si.AxisNum+6);       % Gripper cmd
    
    if str_time(rn) <= si.timeGripperInputHold * 2     % start time difference is too short (shorter than gripper cmd hold time)
        LTTtime(1) = LTTtime(1) + str_time(rn);
    else    % add another step at the beginning to keep starting time aligned
        DesJntPos = [DesJntPos(1,:); DesJntPos];
        GrpCmd = [GrpCmd(1,:); GrpCmd];
        LTTtime = [str_time(rn); LTTtime];
    end       
    
    % Generate external command and download to target PC
    ReplayTime = LTTtime / opt.speedup; % sec
    extcmd = ExtCmdGenerate(DesJntPos, ReplayTime, si, rsi{rn}, GrpCmd, opt);
    ExtCmdDownload(ParamSgnID{rn}, extcmd);    
end

%% Execute the trajectory on target PC
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(ParamSgnID, 'off');

% move to initial pos first to make sure both robots can start the LTT motion at the same time
tp_pos_run(si, IniJntPos, opt);     
tp_prg_run(si, opt);

brake_on_off(ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);

end
