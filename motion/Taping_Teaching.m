%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       Main File for Taping Task Teaching
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/08/25
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Here it is assumed that Robot 2 holds taping tool while Robot 1 is to
% grab the cable and also push the cutting button to cut the tape.
% Teach Robot 1 by Force LTT, Teach Robot 2 by Mouse LTT.

function Taping_Teaching(si, hObject)

persistent wasStopped wasBrakeOff;

robot_idx_FT = 1;   % use force teaching for robot 1 (grab cable)
robot_idx_MT = 2;   % use mouse teaching for robot 2 (grab taping tool)

ParamSgnID = si.ParamSgnID(robot_idx_FT);
num = numel(ParamSgnID);
tg = SimulinkRealTime.target;

if isempty(wasStopped)
    wasStopped = 1;
    wasBrakeOff = zeros(num, 1);
end

if get(hObject,'Value')     % start teaching
    set(hObject,'String', 'Finish Taping Teaching');
    set(hObject,'ForegroundColor', [1,0,0]);

    wasStopped = tg_start_stop('start');
    wasBrakeOff = brake_on_off(ParamSgnID, 'off');
    
    gripper_action(ParamSgnID, 'DRIVE', cellstr(repmat('fully open', numel(ParamSgnID), 1)));  

    for rn = 1:num
        setparam(tg, ParamSgnID{rn}.LTT_Run, 1);
    end
    % set robot 1 to grab cable firmly when robot 2 is doing circling
    setparam(tg, ParamSgnID{1}.DES_MODE, gripper_mode('grab cable firmly'));

    % set robot 2 with mouse teaching mode to grab the taping tool
    opt.robot_idx = robot_idx_MT;
    Mouse_Teaching(si, opt);
    
else                        % finish teaching
    set(hObject,'String', 'Start Taping Teaching');
    set(hObject,'ForegroundColor', [0,0,1]);
    
    for rn = 1:num
        setparam(tg, ParamSgnID{rn}.LTT_Run, 0);
    end
    
    brake_on_off(ParamSgnID, 'on', wasBrakeOff);
    tg_start_stop('stop', wasStopped);
    
    % save LTT record data without replay
    opt.DataMode = 1;
    LTT_Motion_Planning(si, opt);
end
