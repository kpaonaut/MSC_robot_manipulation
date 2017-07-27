%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       Main File for Taping Task Teaching
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/08/25
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function LTT_Teaching(si, hObject, opt)

persistent wasStopped wasBrakeOff;

if nargin < 3,  opt = [];  end
if ~isfield(opt,'robot_idx') || isempty(opt.robot_idx),  opt.robot_idx = 1:si.robot_num;  end   % robot index set for which operations should be conducted (default: all robots)
if ~isfield(opt,'DataMode') || isempty(opt.DataMode),  opt.DataMode = 0;  end   % 0 - do not save teaching data (default); 1 - save new LTT data but no replay

ParamSgnID = si.ParamSgnID(opt.robot_idx);
num = numel(ParamSgnID);
tg = SimulinkRealTime.target;

if isempty(wasStopped)
    wasStopped = 1;
    wasBrakeOff = zeros(num, 1);
end

if get(hObject,'Value')     % start teaching
    set(hObject,'String', 'Finish LTT Teaching');
    set(hObject,'ForegroundColor', [1,0,0]);

    wasStopped = tg_start_stop('start');
    wasBrakeOff = brake_on_off(ParamSgnID, 'off');
    
    for rn = 1:num
        setparam(tg, ParamSgnID{rn}.LTT_Run, 1);
    end
    
    % Here it is assumed that Robot 2 holds taping tool while Robot 1 is to
    % grab the cable and also push the cutting button to cut the tape.
    setparam(tg, ParamSgnID{1}.DES_MODE, gripper_mode('grab cable firmly'));
    if si.robot_num > 1,  setparam(tg, ParamSgnID{2}.DES_MODE, gripper_mode('grab cable taping tool firmly'));  end
    
else                        % finish teaching
    set(hObject,'String', 'Start LTT Teaching');
    set(hObject,'ForegroundColor', [0,0,1]);
    
    for rn = 1:num
        setparam(tg, ParamSgnID{rn}.LTT_Run, 0);
    end
    
    brake_on_off(ParamSgnID, 'on', wasBrakeOff);
    tg_start_stop('stop', wasStopped);
    
    % save LTT record data without replay
    if opt.DataMode
        LTT_Motion_Planning(si, opt);
    end
end
