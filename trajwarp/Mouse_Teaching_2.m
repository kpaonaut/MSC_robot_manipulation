%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       Main File for Two Robot Mouse Teaching
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/08/25
%  Modified by Te Tang, 09/28/2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function Mouse_Teaching_2(si, hObject)

persistent wasStopped wasBrakeOff;

tg = SimulinkRealTime.target;

if isempty(wasStopped)
    wasStopped = 1;
    wasBrakeOff = [0;0];
end

if get(hObject,'Value')     % start teaching
    set(hObject,'String', 'Finish Mouse Teaching 2');
    set(hObject,'ForegroundColor', [1,0,0]);

    wasStopped = tg_start_stop('start');
    wasBrakeOff = brake_on_off(si.ParamSgnID, 'off');
    
    gripper_action(si.ParamSgnID(1), 'DRIVE', cellstr(repmat('fully open', 1, 1)));  

    % set robot 1 to grab cable firmly
    setparam(tg, si.ParamSgnID{1}.DES_MODE, gripper_mode('grab cable firmly'));

    % both robot can use Mouse teaching
    Mouse_Teaching_TwoRobot(si);
    
else                        % finish teaching
    set(hObject,'String', 'Start Mouse Teaching 2');
    set(hObject,'ForegroundColor', [0,0,1]);
    
    brake_on_off(si.ParamSgnID, 'on', wasBrakeOff);
    tg_start_stop('stop', wasStopped);
    
    % save LTT record data without replay
    opt.DataMode = 1;
    LTT_Motion_Planning(si, opt);
    
end
