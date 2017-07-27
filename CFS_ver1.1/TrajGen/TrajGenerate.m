%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%       Generate external motion command from specified points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%  Based on Wenjie's work in FANUC Robot Experimentor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Parameter Definition:
   % qr = [q1;q2;q3; q4]; % deg
   % TimeVec;   %[2,2,2];  %unit: sec
% output external command format: extcmd (see below)

function traj = TrajGenerate(qr, TimeVec, si, rsi, grpcmd, opt)

if nargin < 5 || isempty(grpcmd)
%     cur_grpcmd = gripper_action(si.ParamSgnID(rsi.robot_no), 'GETALL');
%     grpcmd = repmat(cur_grpcmd{1}, size(qr,1), 1);  
    grpcmd = repmat(zeros(1,6), size(qr,1), 1); 
end

if nargin < 6,  opt = [];  end
if ~isfield(opt,'interpType') || isempty(opt.interpType),  opt.interpType = 'joint';  end   % interpolation at: 'joint' space (default); 'TCP' space; 'linear' interpolation
if ~isfield(opt,'SI_jnt') || isempty(opt.SI_jnt),  opt.SI_jnt = 0;  end   % 0 - degree (default); 1 - radian
if ~isfield(opt,'LTT') || isempty(opt.LTT),  opt.LTT = 0;  end   % 0 - not in LTT mode (default); 1 - in LTT mode
if ~isfield(opt,'time_grp_action') || isempty(opt.time_grp_action),  opt.time_grp_action = 1;  end   % time to wait for gripper to finish action, unit - sec
if ~isfield(opt,'mstraj') || isempty(opt.mstraj),  opt.mstraj = 0;  end   % 0 - no mstraj (default); 1 - mstraj


%% Define Time Inteval
ITP = si.itp*1000;     % itp time[ms]    0.008*1000 = 8
TimeStep = floor(TimeVec/si.itp);
num_step = length(TimeStep);
time_step_wait_gripper = floor(opt.time_grp_action/si.itp);
opt.interp = 0;     % for not to do interpolation in fanucikine (since interpolation is done here)

%% Joint Trajectory Generation
q = [];  dq = [];  g_cmd = [];
% first should make sure smooth continuity
if opt.SI_jnt 
    qr = unwrap(qr);
else
    qr = rad2deg(unwrap(deg2rad(qr)));
end

% if strcmpi(opt.interpType, 'tcp')
%     if ~opt.mstraj
%         [~, T] = fanucfkine(qr, rsi, opt);
%     else
%         [xyzwpr, ~] = fanucfkine(qr, rsi, opt);
% %         xyzwpr(:,1:3) = xyzwpr(:,1:3)./1000;
% %         xyzwpr(:,4:6) = xyzwpr(:,4:6).*(pi/180);
%     end
% end
if strcmpi(opt.interpType, 'tcp'),  [~, T] = fanucfkine(qr, rsi, opt);  end

if opt.mstraj
    if(~isfield(opt,'TaccRatio') || isempty(opt.TaccRatio))  
    tacc_ratio = 0.5;
    TaccRatio = tacc_ratio*eye(size(TimeVec,2));
    else
        TaccRatio =opt.TaccRatio;
    end
end
if ~opt.mstraj % ctraj or jtraj
for i = 1:num_step
    % if there is actual gripper command and it is between gripper cmd change period, add time to wait for gripper to finish action
    if TimeStep(i) < floor(si.timeGripperInputHold * 3 / si.itp)  
        if sum(grpcmd(i,2:5)) > 0   % add time during setup/hold/drive/reset state
            TimeStep(i) = time_step_wait_gripper;
        else    % skip this redundant step and continue to next step
            continue;
        end
    end
    
    if strcmpi(opt.interpType, 'tcp')   % tcp interpolation
        tc_temp = ctraj(T(:,:,i),T(:,:,i+1),TimeStep(i));
        q_temp = fanucikine(tc_temp, rsi, qr(i,:), opt);
        dq_temp = diff2(q_temp)/si.itp;
    elseif strcmpi(opt.interpType, 'joint')    % joint interpolation
        time_temp = (0:ITP:(TimeStep(i)-1)*ITP)';
        [q_temp, dq_temp, ~] = jtraj(qr(i,:), qr(i+1,:), time_temp/1000); 
    else    % linear interpolation (for already planned continuous smooth trajectory)
        if i == 1
            qd0 = zeros(1, size(qr,2));     % for smooth starting
        else
            qd0 = dq(end,:);    % for continouity   % (qr(i+1,:) - qr(i,:)) / (TimeStep(i) - 1) / si.itp;
        end
        
        if i == num_step
            qdf = zeros(1, size(qr,2));     % for smooth ending
        else
            qdf = (qr(i+1,:) - qr(i,:)) / (TimeStep(i) - 1) / si.itp;   % as linear constant velocity
        end
        
        time_temp = (0:ITP:(TimeStep(i)-1)*ITP)';
        [q_temp, dq_temp, ~] = jtraj(qr(i,:), qr(i+1,:), time_temp/1000, qd0, qdf);   
        
    end
    
    q = vertcat(q, q_temp);
    dq = vertcat(dq, dq_temp);
    g_cmd = vertcat(g_cmd, repmat(grpcmd(i,:), size(q_temp,1)-1, 1));   % keep current gripper command until the last one step
    g_cmd = vertcat(g_cmd, grpcmd(i+1,:));  % start to use the next gripper command from the last one step    
end

else  % Use mstraj
%     Tacc = rsi.AccelTime2_s.*si.itp;
    Tacc = TaccRatio*TimeVec';
    if strcmpi(opt.interpType, 'tcp')   % tcp interpolation
        tc_temp = mstraj(xyzwpr(2:end,:), [], TimeVec',qr(1,:),si.itp,Tacc);
        q = fanucikine(tc_temp, rsi, qr(1,:), opt);
        dq = diff2(q)/si.itp;
    else strcmpi(opt.interpType, 'joint')    % joint interpolation
        q = mstraj(qr(2:end,:),[],TimeVec',qr(1,:),si.itp,Tacc); 
        dq = diff2(q)/si.itp; 
    % No need for 'linear' 
    end    
end




if strcmpi(opt.interpType, 'tcp')   % for tcp interpolation, again should make sure smooth continuity
    q(1, :) = qr(1, :);     % make sure to start from the current position
    if opt.SI_jnt
        q = unwrap(q);
    else
        q = rad2deg(unwrap(deg2rad(q)));      
    end
    dq = diff2(q)/si.itp;
end

if opt.LTT  % if in LTT mode, smooth the position trajectory and then diff again
    [b, a] = butter(4, 5*2*si.itp);  % 5Hz 4-th order low pass filter
    q = filtfilt(b, a, q);  
    dq = diff2(q)/si.itp;
end

if ~opt.SI_jnt, dq = deg2rad(dq);  end      % change to rad/sec

jntvel = dq;                                % Joint VelCmd [rad/s]
dq = (rsi.interaction\dq')';                % Motor VelCmd [rad/s]

%% Write into FANUC external command format
ang_int = qr(1,:);
if opt.SI_jnt,  ang_int = rad2deg(ang_int);  end    % change to deg

vcmd_length = size(dq,1);
tmp_zeros = zeros(si.ext_vel_max_cnt-vcmd_length, 6);

traj.vcmd_t = [dq*(2^20/(2*pi)*si.itp); tmp_zeros];   % motor counts
traj.vcmd_EndCnt_t = vcmd_length;
traj.inipos_t = ang_int;      % degree
% extcmd.grippercmd = [g_cmd; tmp_zeros];

%%
traj.q = q;
traj.dq = jntvel;
traj.t = (0:ITP:(sum(TimeStep)-1)*ITP)'./1000;
if opt.mstraj ==1, traj.t = traj.t(1:length(q)); end
fprintf('-----> Complete external command generation.\n')
