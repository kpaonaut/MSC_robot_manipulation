%% Wenjie Chen, 2016/06/24, FANUC Corporation
% Do system identification for force sensor (bias and payload parameters)
% Based on Hsien-Chung Lin's work during 2015 summer internship

function FSparams = FT_sensor_identification(si, opt)

if nargin < 2,  opt = [];  end
if ~isfield(opt,'robot_idx') || isempty(opt.robot_idx),  opt.robot_idx = 1:si.robot_num;  end   % robot index set for which operations should be conducted (default: all robots)
if ~isfield(opt,'relPath2Data') || isempty(opt.relPath2Data),  opt.relPath2Data = 'data\';  end
if ~isfield(opt,'fname_prefix') || isempty(opt.fname_prefix),  opt.fname_prefix = 'FS_ID_Data_';  end  % Define the filename prefix for FS ID data file
if ~isfield(opt,'DataMode') || isempty(opt.DataMode),  opt.DataMode = 1;  end   % dafault: 1 - do new exp to get data

rsi = si.ri(opt.robot_idx);     ParamSgnID = si.ParamSgnID(opt.robot_idx);

% Initialization
FSparams = cell(size(opt.robot_idx));
loadfile = cell(size(opt.robot_idx));
for rn = 1:numel(opt.robot_idx)
    loadfile{rn} = ['POS_FT', num2str(rsi{rn}.robot_no)];
end

if opt.DataMode == 1    % Do a new exp to get new data

    fprintf('---> Now starting a new experiment test to get ID dataset...\n\n');
    
    %% Generate Motion Trajectory and download to target PC
    for rn = 1:numel(opt.robot_idx)
        disp(['---> Now generating motion commands for Robot No. ', num2str(rsi{rn}.robot_no), '...']);
        
        q = [rsi{rn}.iniJntPos(1), 0, 0, 0, -90, 0;...
            rsi{rn}.iniJntPos(1), 0, 0, 90, -90, 0;...
            rsi{rn}.iniJntPos(1), 0, 0, -90, -90, 0;...
            rsi{rn}.iniJntPos(1), 0, 0, 0, 0, 0;...
            rsi{rn}.iniJntPos(1), 0, 0, 0, 0, 90;...
            rsi{rn}.iniJntPos(1), 0, 0, 0, 0, -90;...
            rsi{rn}.iniJntPos(1), 0, 0, 0, -90, 0];
        qVec = [];  tVec = [];
        for i = 1:size(q,1)
            qVec = [qVec; q(i,:); q(i,:)];
            tVec = [tVec, 2, 8];    % use 2 sec to stay at the same pos, 8 sec to move to the next pos
        end
        tVec = tVec(1:end-1);      % unit: sec

        % Generate external command and download to target PC
        extcmd = ExtCmdGenerate(qVec, tVec, si, rsi{rn});
        ExtCmdDownload(ParamSgnID{rn}, extcmd);    
    end
    
    %% Execute the trajectory on target PC
    tp_prg_run(si, opt);
    
    %% Get experimental data from target PC and setup data ready for identification
    ExpData = DataLog(loadfile, opt);
    
else    % Use previous FS ID data
    fprintf('---> Now loading the previous ID dataset.\n\n');
    fList = dir([opt.relPath2Data, opt.fname_prefix, '*.mat']);
    ExpData = load([opt.relPath2Data, fList(end).name]);
end

%% Data processing to compute the parameters
for rn = 1:numel(opt.robot_idx)
    %% Data pre-processing
    disp(['---> Now processing the identification for Robot No. ', num2str(rsi{rn}.robot_no), '...']);
    data = downsample(ExpData.(loadfile{rn}),10);
    
    F = data(:,10:15);          % force/torque data
    q = deg2rad(data(:,1:6));   % joint position angle data
    t = data(:,16);             % time stamps
    
    % get the index when robot is not in motion and exclude the first 5 sec
    % during which the application may have not begun at target PC
    q_diff = [diff(q); zeros(1,6)];
    sum_q_diff = sum(abs(q_diff),2);
    idx = (t>5) & (sum_q_diff < 1e-5);
    figure; plot(t(idx), sum_q_diff(idx));
    title(['Valiad Data Index (Robot No. ', num2str(rsi{rn}.robot_no), ' in static status)'], 'fontsize', 14);
    
    F = F(idx,:);
    q = q(idx,:);
    t = t(idx,:);
    N = size(q,1);
    disp(['---> Number of valid data samples: ', num2str(N)]);
    
    % get the RTB model for rotation matrix computing
    local_rtb = si.rtb{opt.robot_idx(rn)};
    
    %% Least squares identification
    % see Hsien-Chung's 2015 internship presentation ppt for details
    disp('---> Now computing the bias and payload parameters ...');
    
    Af = [];  Bf =[];
    T = local_rtb.fkine(q);
    for i = 1:N
        R = T(1:3, 1:3, i);
        Af = vertcat(Af, [R(3,:)', eye(3)]);
        Bf = vertcat(Bf, F(i,1:3)');
    end
    
    X = Af\Bf;
    mg = X(1);
    Fb = X(2:end);      % bias in force measurement
    Fg = [0; 0; mg];    % payload gravity force
    
    At = [];  Bt = [];
    for i = 1:N
        R = T(1:3, 1:3, i);
        Fgs = R' * Fg;
        G = [0, Fgs(3), -Fgs(2);
            -Fgs(3), 0, Fgs(1);
            Fgs(2), -Fgs(1), 0];
        At = vertcat(At, [G, eye(3)]);
        Bt = vertcat(Bt, F(i,4:6)');
    end
    
    Y = At\Bt;
    r = Y(1:3);     % payload center of gravity in tool frame
    Tb = Y(4:6);    % bias in torque measurement
    
    %% Check the residual error
    Fcomp = zeros(numel(t),3);  Tcomp = zeros(numel(t),3);
    for i = 1:N
        R = T(1:3, 1:3, i);
        Fcomp(i,:) = F(i,1:3) - (R' * Fg)'- Fb';
        Tcomp(i,:) = F(i,4:6) - cross(r, R' * Fg)'-Tb';
    end
    
    figure;
    subplot(3,1,1); plot(t,F(:,1),'b--',t,Fcomp(:,1),'r'); grid on
    ylabel('Fx(N)'); xlabel('time(sec)')
    title(['Residual Force Errors for Robot No. ', num2str(rsi{rn}.robot_no)], 'fontsize', 14);
    legend('Before Comp', 'After Comp');
    subplot(3,1,2); plot(t,F(:,2),'b--',t,Fcomp(:,2),'r'); grid on
    ylabel('Fy(N)'); xlabel('time(sec)')
    subplot(3,1,3); plot(t,F(:,3),'b--',t,Fcomp(:,3),'r'); grid on
    ylabel('Fz(N)'); xlabel('time(sec)')
    
    figure;
    subplot(3,1,1); plot(t,F(:,4),'b--',t,Tcomp(:,1),'r'); grid on
    ylabel('Tx(Nm)'); xlabel('time(sec)')
    title(['Residual Torque Errors for Robot No. ', num2str(rsi{rn}.robot_no)], 'fontsize', 14);
    legend('Before Comp', 'After Comp');
    subplot(3,1,2); plot(t,F(:,5),'b--',t,Tcomp(:,2),'r'); grid on
    ylabel('Ty(Nm)'); xlabel('time(sec)')
    subplot(3,1,3); plot(t,F(:,6),'b--',t,Tcomp(:,3),'r'); grid on
    ylabel('Tz(Nm)'); xlabel('time(sec)')
    
    %% Return identified values
    FSparams{rn}.Fb = Fb;   % bias in force measurement
    FSparams{rn}.Tb = Tb;   % bias in torque measurement
    FSparams{rn}.Fg = Fg;   % payload gravity force
    FSparams{rn}.r = r;     % payload center of gravity in tool frame
    disp(['-----> Bias in Force Measurement    (N)  :  ', num2str(Fb')]);
    disp(['-----> Bias in Torque Measurement   (Nm) :  ', num2str(Tb')]);
    disp(['-----> Payload Gravity Force        (N)  :  ', num2str(Fg')]);
    disp(['-----> Payload C of G in Tool Frame (m)  :  ', num2str(r')]);
    fprintf(['---> System identification for F/T sensor and payload (Robot No. ', num2str(rsi{rn}.robot_no), ') is finished. \n\n']);
    
end

error('---> Please update the F/T sensor parameters in init_LRMate200_Dual.m and then re-build the application.');
