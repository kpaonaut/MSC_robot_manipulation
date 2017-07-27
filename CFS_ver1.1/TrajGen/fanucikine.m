%% Wenjie Chen, 2016/07/04, FANUC Corporation
% Inverse kinematics wrapper function for using FANUC rlib functions

function JntPos = fanucikine(xyzwpr, rsi, last_jnt, opt)
% Input:    xyzwpr - m*6 matrix, m is number of sets, default unit: mm, deg
%                    could also be 4*4*m transformation matrix, default unit: m
%           last_jnt - 1*n vector, n is joint number, m is number of sets, RTB convetion, default unit: deg
% Output:   JntPos - m*n matrix, n is joint number, m is number of sets, RTB convetion, default unit: deg

if nargin < 4,  opt = [];  end
if ~isfield(opt,'SI_jnt') || isempty(opt.SI_jnt),  opt.SI_jnt = 0;  end   % 0 - degree (default); 1 - radian
if ~isfield(opt,'SI_tcp') || isempty(opt.SI_tcp),  opt.SI_tcp = 0;  end   % 0 - degree, mm (default); 1 - radian, m
if ~isfield(opt,'interp') || isempty(opt.interp),  opt.interp = 1;  end   % 1 - do interpolation (default); 0 - not to do interpolation;
if ~isfield(opt,'cfg_tol_tran') || isempty(opt.cfg_tol_tran),  opt.cfg_tol_tran = 30;  end   % tolerance of translation for using same cfg, unit - mm
if ~isfield(opt,'cfg_tol_rot') || isempty(opt.cfg_tol_rot),  opt.cfg_tol_rot = 10;  end   % tolerance of rotation for using same cfg, unit - deg
if ~isfield(opt,'is_recursive') || isempty(opt.is_recursive),  opt.is_recursive = 0;  end   % 0 - not in recursive loop (default); 1 - in recursive loop

last_jnt_raw = last_jnt;    % save last_jnt for input of recursive iteration

if (size(xyzwpr,1) == 4) && (size(xyzwpr,2) == 4)   % if xyzwpr is transformation matrices
    T = xyzwpr;
    xyzwpr = zeros(size(T, 3), 6);
    xyzwpr(:,1:3) = transl(T) * 1000;           % unit - mm
    xyzwpr(:,6:-1:4) = tr2rpy(T, 'deg', 'zyx'); % unit - deg
    opt.SI_tcp = 0;
end
if size(xyzwpr,2) ~= 6,  xyzwpr = xyzwpr';  end
if size(xyzwpr,2) ~= 6,  error('Input TCP position dimension does not equal to 6!');  end

if ~opt.SI_jnt,  last_jnt = deg2rad(last_jnt);  end;
if opt.SI_tcp,  xyzwpr(:,4:6) = rad2deg(xyzwpr(:,4:6));  xyzwpr(:,1:3) = xyzwpr(:,1:3)*1000;  end;

config_ref = 0;  % 0: WCP config, 1: TCP config
choice = 0;      % 0: FULL solution, 1: HALF solution
num = size(xyzwpr, 1);
JntPos = zeros(num, rsi.n);
last_jnt0 = last_jnt;    % save last_jnt for final unwrap process

% Convert to FANUC joint angle convention (J3 angle is relative angle from horizontal plane)
last_jnt(3) = last_jnt(3) - last_jnt(2);
for i = 1 : num
    [last_xyzwpr, i_cfg, ~] = rlfklr2d7l(last_jnt, rsi.utool, config_ref);    % last_jnt - rad;  utool - mm
    
    % compute the distance from last position to desired position
    if opt.interp && ~opt.is_recursive   % only do one level recursion
        last_R = rpy2r(last_xyzwpr(6:-1:4), 'deg', 'zyx');
        dest_R = rpy2r(xyzwpr(i,6:-1:4), 'deg', 'zyx');
        dif_theta = 2 * acosd(Quaternion(last_R' * dest_R).s);
        dif_theta = min(abs(dif_theta), abs(dif_theta - 360));
        dif_tran = norm(last_xyzwpr(1:3) - xyzwpr(i, 1:3));
        steps = max(floor(dif_tran / opt.cfg_tol_tran), floor(dif_theta / opt.cfg_tol_rot)) + 2;
    else
        steps = 2;  % the starting and the ending points are already 2 steps
    end
    
    if steps > 2 && ~opt.is_recursive   % only do one level recursion
        % If the distance is too far, it is not good to use cfg directly
        % for ik at desired position. Try to do cart interpolation along
        % the trajectory to shorten each step's difference.
        last_T = [last_R, last_xyzwpr(1:3)'/1000; 0 0 0 1];     % unit - m
        dest_T = [dest_R, xyzwpr(i, 1:3)'/1000; 0 0 0 1];       % unit - m
        tc_temp = ctraj(last_T, dest_T, linspace(0,1,steps));
        opt.is_recursive = 1;   % start recursive iteration
        q_temp = fanucikine(tc_temp, rsi, last_jnt_raw, opt);
        opt.is_recursive = 0;   % finish recursive iteration
        JntPos(i, :) = q_temp(end, :);
    else
        % Use cfg from last position directly for ik at desired position
        [JntPos(i,:), ~] = rliklr2d7l(xyzwpr(i,:), i_cfg, rsi.utool, last_jnt, choice);  % JntPos - rad;  xyzwpr - mm, deg;  last_jnt - rad;  utool - mm
    end
    last_jnt = JntPos(i,:);
end

if ~opt.is_recursive    % if not in recursive loop (will be used as output)
    % Convert from FANUC joint angle convention (J3 angle is relative angle from horizontal plane)
    JntPos(:,3) = JntPos(:,3) + JntPos(:,2);
    % Make smooth continous trajectory from last_jnt
    JntPos = unwrap([last_jnt0; JntPos]);    
    JntPos = JntPos(2:end, :);     
    if ~opt.SI_jnt,  JntPos = rad2deg(JntPos);  end;
end
