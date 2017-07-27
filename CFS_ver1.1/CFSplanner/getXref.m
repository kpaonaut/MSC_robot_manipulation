%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%     get vectorized reference for CFS algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%  Based on Changliu's CFS algorithm 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Input:
%  robot   : [1x2 cell], dual robot parameters
%  status  : [1x1 struct], identify the moving/static robot
%  id      : [1x1 struct], indicate the submotion
% 
%  Output:
%  xref    : [Nstate*horizon x1], the vectorized downsample position reference
%  horizon : [1x1], the total horizon steps number
%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [xref, horizon] = getXref(robot, status, id)
MoID = id.MoID;
SubMoID = id.SubMoID;
nlink = robot{1}.nlink;

if strcmp(status.dualmotion,'yes')
    traj{1} = robot{1}.motion{MoID}.traj{SubMoID}.xref;
    traj{2} = robot{2}.motion{MoID}.traj{SubMoID}.xref;
    horizon = size(traj{1},1)/(robot{1}.nlink);
    
    xref = zeros(size(traj{1},1)+size(traj{2},1),1);
    for i = 1:horizon
        xref((i-1)*nlink*2+1:i*nlink*2) = [traj{1}((i-1)*nlink+1:i*nlink);
                                             traj{2}((i-1)*nlink+1:i*nlink)];
    end

    if size(xref,2)>1, xref = xref'; end
    
else
    xref =robot{status.MovingRobot}.motion{MoID}.traj{SubMoID}.xref;
    if size(xref,2)>1, xref = xref'; end
    horizon = size(xref,1)/robot{status.MovingRobot}.nlink;
end