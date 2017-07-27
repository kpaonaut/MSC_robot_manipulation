%% Wenjie Chen, 2016/07/04, FANUC Corporation
% Forward kinematics wrapper function for using FANUC rlib functions

function [xyzwpr, T] = fanucfkine(JntPos, rsi, opt)
% Input:    JntPos - m*n matrix, n is joint number, m is number of sets, default unit: deg
% Output:   xyzwpr - m*6 matrix, m is number of sets, default unit: mm, deg
%           T      - 4*4*m matrix, m is number of sets, default unit: m

if nargin < 3,  opt = [];  end
if ~isfield(opt,'SI_jnt') || isempty(opt.SI_jnt),  opt.SI_jnt = 0;  end   % 0 - degree (default); 1 - radian
if ~isfield(opt,'SI_tcp') || isempty(opt.SI_tcp),  opt.SI_tcp = 0;  end   % 0 - degree, mm (default); 1 - radian, m
if ~isfield(opt,'is_dual_arm') || isempty(opt.is_dual_arm),  opt.is_dual_arm = evalin('base', 'opt.is_dual_arm');  end   % check if it is dual arm

if size(JntPos,2) ~= rsi.n,  JntPos = JntPos';  end
if size(JntPos,2) ~= rsi.n,  error('Input joint position dimension does not correspond to robot joint number!');  end
if ~opt.SI_jnt,  JntPos = deg2rad(JntPos);  end;

config_ref = 0;  % 0: WCP config, 1: TCP config
num = size(JntPos, 1);
xyzwpr = zeros(num, 6);

% Rotation angle RPY about the moving frame ZYX from world frame to design
% space frame. This is the same rotation as the FANUC convetion WPR (about 
% the fixed frame XYZ), with vector order revsered. Use rpy2r(RPY, 'zyx')
% to compute the rotation matrix.
% Convert to FANUC joint angle convention (J3 angle is relative angle from horizontal plane)
JntPos(:,3) = JntPos(:,3) - JntPos(:,2);
for i = 1 : num
    % xyzwpr - mm, deg;       JntPos - rad;       utool - mm
    if opt.is_dual_arm  % currently dual arm uses LR Mate 200iD/7L robots
        [xyzwpr(i,:), ~, ~] = rlfklr2d7l(JntPos(i,:), rsi.utool, config_ref);
    else    % single arm uses LR Mate 200iD robot
        [xyzwpr(i,:), ~, ~] = rlfklr2d(JntPos(i,:), rsi.utool, config_ref);
    end
end

if opt.SI_tcp,  xyzwpr(:,4:6) = deg2rad(xyzwpr(:,4:6));  xyzwpr(:,1:3) = xyzwpr(:,1:3)/1000;  end;
T = xyzwpr2T(xyzwpr, opt);
