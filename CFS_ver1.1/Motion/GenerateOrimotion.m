%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%    Concatenate all the submotion xref of a single motion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%  Input:
%  motion: [1x1 struct], a robot motion
%  varagin: if added, should be concatenate object. (same struct as output "orimotion")

%  Output:
%  orimotion: [Nx6] the robot joint trajectory
%  mostep   : [1x1] total motion step
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [orimotion, mostep] = GenerateOrimotion(motion, varargin)

if isempty(varargin), 
    orimotion =[];
else
    orimotion = varargin{1};
end

traj = motion.traj;

xref = [];

for i = 1:length(traj)
    xref = [xref; traj{i}.xref];
end

Xref = reshape(xref, 6, length(xref)/6);
temp = Xref';
orimotion = [orimotion; temp];
mostep = size(orimotion,1); % this is for visualization pause option