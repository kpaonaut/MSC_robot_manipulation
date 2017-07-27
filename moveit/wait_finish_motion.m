%% Wenjie Chen, 2016/07/04, FANUC Corporation
% Shortcut of sequences to wait until motions are finished

function wait_finish_motion(ParamSgnID)

tg = SimulinkRealTime.target;
pause(0.3);     % wait for the mode controller to start executing motion

motionFinished = false;
while ~motionFinished
    motionFinished = true;
    for rn = 1:numel(ParamSgnID)
        if ismember(getsignal(tg, ParamSgnID{rn}.M_State), [9931, 31, 33, 40, 21, 22, 9921:9928])
            % in state: TpPosRun, TpPrgRun (ini, Run, Hold), Jog, JogC, ForceRun, LTT, TI
            motionFinished = false;
        end
    end
    pause(0.2);    
end

disp('-----> Current motion task is finished.');

end
