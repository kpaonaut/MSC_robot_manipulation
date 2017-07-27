%% Wenjie Chen, 2016/07/05, FANUC Corporation
% Shortcut of sequences to execute gripper actions

function curCmd = gripper_action(ParamSgnID, action, value)

tg = SimulinkRealTime.target;
if ~iscell(ParamSgnID),  ParamSgnID = {ParamSgnID};  end 
num = numel(ParamSgnID);

if strcmpi(action, 'GETALL')
    curCmd = cell(num,1);
    for rn = 1:num
        curCmd{rn}(1) = getparam(tg, ParamSgnID{rn}.GPR_MODE);
        curCmd{rn}(2) = getparam(tg, ParamSgnID{rn}.GPR_SETUP);
        curCmd{rn}(3) = getparam(tg, ParamSgnID{rn}.GPR_HOLD);
        curCmd{rn}(4) = getparam(tg, ParamSgnID{rn}.GPR_DRIVE);
        curCmd{rn}(5) = getparam(tg, ParamSgnID{rn}.GPR_RESET);
        curCmd{rn}(6) = getparam(tg, ParamSgnID{rn}.GPR_SVON);
    end
    return;
end

disp(['---> Now starting to execute gripper action : ', action]);
wasStopped = tg_start_stop('start');

if strcmpi(action, 'RESET')
    for rn = 1:num
        setparam(tg, ParamSgnID{rn}.GPR_RESET, 1);
        while (~getsignal(tg, ParamSgnID{rn}.GPR_ALARM) || ~getsignal(tg, ParamSgnID{rn}.GPR_ESTOP)),  pause(0.1);  end
        setparam(tg, ParamSgnID{rn}.GPR_RESET, 0);
        disp(['-----> Gripper No. ', num2str(ParamSgnID{rn}.robot_no), ' is RESET.']);
    end
end

% For the following operations, first check if there is any alarm/estop
for rn = 1:num
    if (~getsignal(tg, ParamSgnID{rn}.GPR_ALARM) || ~getsignal(tg, ParamSgnID{rn}.GPR_ESTOP))
        disp(['-----> There is alarm/E-Stop on Gripper No. ', num2str(rn), '. The gripper is now being reset.']);
        gripper_action(ParamSgnID, 'RESET');
    end
end
    
if strcmpi(action, 'SVON')
    for rn = 1:num
        setparam(tg, ParamSgnID{rn}.GPR_SVON, 1);
    end
    % this may take up to 10 sec when the gripper is just powered on
    disp('-----> Waiting gripper servo to be on ...');
    for rn = 1:num
        while (~getsignal(tg, ParamSgnID{rn}.GPR_SVRE)),  pause(0.1);  end
        disp(['-----> Gripper No. ', num2str(ParamSgnID{rn}.robot_no), ' is SVON.']);
    end
end

if strcmpi(action, 'SVOFF')
    for rn = 1:num
        setparam(tg, ParamSgnID{rn}.GPR_SVON, 0);
    end
    for rn = 1:num
        while (getsignal(tg, ParamSgnID{rn}.GPR_SVRE)),  pause(0.1);  end
        disp(['-----> Gripper No. ', num2str(ParamSgnID{rn}.robot_no), ' is SVOFF.']);
    end
end

if strcmpi(action, 'SETUP')
    for rn = 1:num
        setparam(tg, ParamSgnID{rn}.GPR_SETUP, 1);
    end
    pause(0.1);     % Hold the IO to let gripper take this input (should be at least 30ms)
    % this may take about 3 sec
    disp('-----> Waiting gripper setup to be finished ...');
    for rn = 1:num       
        while (getsignal(tg, ParamSgnID{rn}.GPR_BUSY) || ~getsignal(tg, ParamSgnID{rn}.GPR_SETON) || ~getsignal(tg, ParamSgnID{rn}.GPR_INP)),  pause(0.1);  end
        setparam(tg, ParamSgnID{rn}.GPR_SETUP, 0);
        disp(['-----> Gripper No. ', num2str(ParamSgnID{rn}.robot_no), ' is SETUP.']);
    end
end

if strcmpi(action, 'DRIVE')
    if nargin < 3,  value = cellstr(repmat('fully open', num, 1));  end      % default mode: fully open
    if ~iscell(value),  value = {value};  end
    
    for rn = 1:num    
        mode{rn} = gripper_mode(value{rn});
        setparam(tg, ParamSgnID{rn}.GPR_MODE, mode{rn});
        pause(ParamSgnID{rn}.timeGripperInputHold);    % Hold the IO to let gripper take this input (should be at least 30ms)
        setparam(tg, ParamSgnID{rn}.GPR_DRIVE, 1);   
    end
    
    pause(0.1);    % Hold the IO to let gripper take this input (should be at least 30ms)

    for rn = 1:num
        while (getsignal(tg, ParamSgnID{rn}.GPR_BUSY)),  pause(0.1);  end
        setparam(tg, ParamSgnID{rn}.GPR_DRIVE, 0);           
        disp(['-----> Gripper No. ', num2str(ParamSgnID{rn}.robot_no), ' is DRIVEN to MODE: ', gripper_mode(mode{rn},0), '.']);
    end
end

if strcmpi(action, 'SETALL')
    if nargin < 3,  error('Need to specify the desired gripper command value!');  end
    if ~iscell(value),  value = {value};  end
    mode = cell(num,1);  setup = zeros(num,1);  drive = zeros(num,1);  reset = zeros(num,1);  svon = zeros(num,1);  
    for rn = 1:num
        mode{rn} = value{rn}(1);
        setup(rn) = value{rn}(2);
        drive(rn) = value{rn}(4);
        reset(rn) = value{rn}(5);
        svon(rn) = value{rn}(6);
    end
    
    idx = find(setup);  if ~isempty(idx),  gripper_action(ParamSgnID(idx), 'SETUP');  end
    idx = find(drive);  if ~isempty(idx),  gripper_action(ParamSgnID(idx), 'DRIVE', mode(idx));  end
    idx = find(reset);  if ~isempty(idx),  gripper_action(ParamSgnID(idx), 'RESET');  end
    idx = find(svon);  if ~isempty(idx),  gripper_action(ParamSgnID(idx), 'SVON');  end
    idx = find(svon == 0);  if ~isempty(idx),  gripper_action(ParamSgnID(idx), 'SVOFF');  end
end

tg_start_stop('stop', wasStopped);

end
