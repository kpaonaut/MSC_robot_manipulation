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

function [Uref, submostep ]= GenerateUref(robot, status, id, uref, varargin)

if isempty(varargin), 
    Uref =[];
else
    Uref = varargin{1};
end

% whem two robots no move, recard as dual robot to save traj
if strcmp(status.move,'no'),   status.dualmotion = 'yes';    end

if strcmp(status.dualmotion,'yes')
    temp{1} = uref(:,1:6);
    temp{2} = uref(:,7:12);
else
    idm = status.MovingRobot; ids = status.StaticRobot;
    temp{idm} = uref;
    temp{ids} = zeros(size(uref,1),6);
end

Uref{1} = [Uref{1}; temp{1}];
Uref{2} = [Uref{2}; temp{2}];

submostep = size(Uref{1},1);