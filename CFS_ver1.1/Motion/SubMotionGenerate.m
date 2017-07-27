%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%   Genetate all the submotions(trajectories) as extcmd
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% This function only generated "traj.extcmd"
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
%               reference.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function motion = SubMotionGenerate(motion,si, ri,gprcmd, opt)
% Generate trajectory from "DesJntPos" and "ReplayTime", currently directly
% each submotion is generated from two DesJntPos and one ReplayTime.


N = length(motion.ReplayTime);
for i = 1:N
    motion.traj{i}.extcmd = TrajGenerate(motion.DesJntPos(i:i+1,:),motion.ReplayTime(i),si,ri,gprcmd,opt);
end

end


% The comment out code is for the data processed by "SubMotionSplit"
% N = size(motion.traj,2);
% for i = 1:N
%     motion.traj{i}.extcmd = ExtCmdGenerate(motion.traj{i}.JntPos,motion.traj{i}.TimeVec,si,ri,gprcmd,opt);
% end