%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%  downsample extcmd and generate the vectorized path reference
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% This function is valid only if "traj.extcmd" is generated.
% This function only generated "traj.xref", 
% 
% Data Format of "motion":
% DesJntPos  : [Nx6], N robot desired joint position  
% ReplayTime : [N-1], N-1 time vector
% GrpCmd     : [Nx6], N robot gripper command
% Marker_T_W : [4x4] or empty,
% MoFlag     : [N-1], indicate whether the robot is moving in each submotion
% traj       : [1xN-1] struct, the submotion, which has "extcmd" and "xref"
% 
% Data Formate of "submotion":
% traj.extcmd : [1x1] struct, similar to extcmd.
% traj.xref   : horizon x 1,  a vectorized downsampled joint position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function motion = XrefGenerate(motion, dsrate)
% dsrate: downsample time
N = size(motion.traj,2); %motion.SubMoFlag
for i = 1:N
    q = deg2rad(motion.traj{i}.extcmd.q);
    xref = downsample(q,dsrate)';
    motion.traj{i}.xref = xref(:);
end

