%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%    Concatenate all new xref into a motion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%  Input:
%  motion : [1x2 cell], a robot motion
%  status : [1x1 struct], identify the moving/static robot in a submotion
%  id     : [1x1 struct], indicate which submotion
%  xnew   : [N x Nstate], a new robot path
%  varagin: if added, should be concatenate object. (same struct as output "newmotion")  
% 
%  Output:
%  newmotion: [1x2 cell] the dual robot new joint trajectory (each of them is Nx6)
%  mostep   : [1x1] total motion step
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [newmotion, submostep ]= GenerateNewmotion(robot, status, id, xnew, varargin)

if isempty(varargin), 
    newmotion =[];
else
    newmotion = varargin{1};
end

% whem two robots no move, recard as dual robot to save traj
if strcmp(status.move,'no'),   status.dualmotion = 'yes';    end

if strcmp(status.dualmotion,'yes')
    temp{1} = xnew(:,1:6);
    temp{2} = xnew(:,7:12);
else
    idm = status.MovingRobot; ids = status.StaticRobot;
    xref = reshape(robot{ids}.motion{id.MoID}.traj{id.SubMoID}.xref, 6, size(xnew,1))';
    temp{idm} = xnew;
    temp{ids} = xref;
end

newmotion{1} = [newmotion{1}; temp{1}];
newmotion{2} = [newmotion{2}; temp{2}];

submostep = size(newmotion{1},1);