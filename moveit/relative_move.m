%% Wenjie Chen, 2016/07/12, FANUC Corporation
% Move relatively in specified frame
% Input:    offset  - relative offset position vector, in one of the following frames (tool, base, world, joint space)
%           orgPos  - the position vector that the movement is relative to, in the same frame as offset, use [] if using current position
           
function relative_move(si, offset, orgPos, opt)

if nargin < 3,  orgPos = [];  end
if nargin < 4,  opt = [];  end
if ~isfield(opt,'robot_idx') || isempty(opt.robot_idx),  opt.robot_idx = 1:si.robot_num;  end   % robot index set for which operations should be conducted (default: all robots)
if ~isfield(opt,'frame') || isempty(opt.frame),  opt.frame = 'tool';  end   % tool (default), base, world, joint space
if ~isfield(opt,'SI_jnt') || isempty(opt.SI_jnt),  opt.SI_jnt = 0;  end   % 0 - degree (default); 1 - radian
if ~isfield(opt,'SI_tcp') || isempty(opt.SI_tcp),  opt.SI_tcp = 0;  end   % 0 - degree, mm (default); 1 - radian, m
if ~isfield(opt,'orgPos') || isempty(opt.orgPos),  opt.orgPos = 'current';  end   % 'current' position (default); 'specified' position
if ~isempty(orgPos),  opt.orgPos = 'specified';  end  % use specified position

if (size(offset,2) ~= si.AxisNum && strcmpi(opt.frame, 'joint')) || (size(offset,2) ~= 6 && ~strcmpi(opt.frame, 'joint'))
    offset = offset';  orgPos = orgPos';
end
if numel(opt.robot_idx) ~= size(offset,1)  || (strcmpi(opt.orgPos, 'specified') && sum(size(offset) ~= size(orgPos)))
    error('Dimension of positions offset, origins, and number of robots do not match with each other !');
end

rsi = si.ri(opt.robot_idx);     ParamSgnID = si.ParamSgnID(opt.robot_idx);

tg = SimulinkRealTime.target;
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(ParamSgnID, 'off');

dest_JntPos = zeros(numel(opt.robot_idx), si.AxisNum);
for rn = 1:numel(opt.robot_idx)
    if strcmpi(opt.orgPos, 'current')
        Cur_JntPos = getsignal(tg, ParamSgnID{rn}.JntPos_deg);  
        if opt.SI_jnt,  Cur_JntPos = deg2rad(Cur_JntPos);  end
    else
        Cur_JntPos = orgPos(rn,:);
    end
    
    if strcmpi(opt.frame, 'joint')
        dest_JntPos(rn, :) = Cur_JntPos + offset(rn,:);
        continue;
    end
    
    [~, T] = fanucfkine(Cur_JntPos, rsi{rn}, opt);
    T_offset = xyzwpr2T(offset(rn,:), opt);
    
    switch lower(opt.frame)
        case 'tool'
            T_dest = T * T_offset;
            
        case 'base'     
            T_dest = eye(4);
            T_dest(1:3, 1:3) = T_offset(1:3, 1:3) * T(1:3, 1:3);
            T_dest(1:3, 4) = T(1:3, 4) + T_offset(1:3, 4);
                        
        case 'world'   
            T_T2W = rsi{rn}.T_B2W * T;
            T_w_dest = eye(4);
            T_w_dest(1:3, 1:3) = T_offset(1:3, 1:3) * T_T2W(1:3, 1:3);
            T_w_dest(1:3, 4) = T_T2W(1:3, 4) + T_offset(1:3, 4);
            T_dest = rsi{rn}.T_B2W \ T_w_dest;
    end
    
    dest_JntPos0 = [Cur_JntPos; fanucikine(T_dest, rsi{rn}, Cur_JntPos, opt)];
    % to make sure smooth continuity
    if opt.SI_jnt
        dest_JntPos0 = unwrap(dest_JntPos0);
    else
        dest_JntPos0 = rad2deg(unwrap(deg2rad(dest_JntPos0)));      
    end
    dest_JntPos(rn, :) = dest_JntPos0(end, :);
end

tp_pos_run(si, dest_JntPos, opt);

brake_on_off(ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);

end
