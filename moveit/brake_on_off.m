%% Wenjie Chen, 2016/07/04, FANUC Corporation
% Shortcut of sequences to do brake_off or brake_on on Target PC

function wasBrakeOff = brake_on_off(ParamSgnID, action, wasBrakeOff)

tg = SimulinkRealTime.target;
if ~iscell(ParamSgnID),  ParamSgnID = {ParamSgnID};  end 
if nargin < 3,  wasBrakeOff = zeros(size(ParamSgnID));  end     % default: was  brake on
pauseNeeded = 0;

for rn = 1:numel(ParamSgnID)
    if strcmpi(action, 'on')     % brake on
        if ~wasBrakeOff(rn)   % if it was on, then turn on
            setparam(tg, ParamSgnID{rn}.Brake_Off, 0);
            disp(['-----> Brake is ON for Robot No. ', num2str(ParamSgnID{rn}.robot_no), '.']);
            pauseNeeded = 1;
        end
    elseif strcmpi(action, 'off')    % brake off
        wasBrakeOff(rn) = getparam(tg, ParamSgnID{rn}.Brake_Off);
        if ~wasBrakeOff(rn)   % if it was on, then turn off
            setparam(tg, ParamSgnID{rn}.Brake_Off, 1);
            disp(['-----> Brake is OFF for Robot No. ', num2str(ParamSgnID{rn}.robot_no), '.']);
            pauseNeeded = 1;
        end
    end
end

if pauseNeeded,  pause(0.5);  end     % wait for the mode controller to take in this input and get ready for next mode

end
