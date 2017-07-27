%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%   directly load robot joint data and save as a LTT data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% Note:
% Origin from "loadTapeCuttingMotion", this is a more generized version for
% future data loading.
% Currently this is set to load "qVec", "tVec", "rbt_pos" from both robot
% qVec        : robot joint pos
% rbt_pos.jnt : two robot init jnt pos
% tVec        : TimeVec
% 
% Data Format of "LTT_Data_Motion":
% DesJntPos{r}  : [Nx6], N robot desired joint position  
% ReplayTime{r} : [N-1], N-1 time vector
% GrpCmd{r}     : [Nx6], N robot gripper command
% Marker_T_W    : [4x4] or empty,
%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function loadGeneralMotion(filepath, filename, ActionName, robot_idx)
data = load([filepath,filename]);


if nargin < 4,  robot_idx = 0;  end   % if not indicate the robot, assume as dual motion

N = size(data.qVec,1);

varname = matlab.lang.makeValidName(['LTT_Data_',ActionName]);   % varaname =  LTT_Data_ActionName

if robot_idx == 1
    eval([varname,'.DesJntPos{1} = data.qVec;']);
    eval([varname,'.DesJntPos{2} = repmat(data.rbt_pos.jnt(2, :),N,1);']);
elseif robot_idx == 2
    eval([varname,'.DesJntPos{2} = data.qVec;']);
    eval([varname,'.DesJntPos{1} = repmat(data.rbt_pos.jnt(1, :),N,1);']);
else
    if iscell(data.qVec) && (length(data.qVec) == 2)
        eval([varname,'.DesJntPos{1} = data.qVec{1};']);
        eval([varname,'.DesJntPos{2} = data.qVec{2};']);
    else
        error('In dual motion, qVec should be a [1x2] cell array.')
    end
end

if length(data.tVec) ~= N-1, error('tVec should be number of joint pos - 1.'); end

eval([varname,'.ReplayTime{1} = data.tVec;']);
eval([varname,'.ReplayTime{2} = data.tVec;']);

if ~isfield(data,'GrpCmd')
    eval([varname,'.GrpCmd{1} = repmat([zeros(1,5), 1],N,1);']);
    eval([varname,'.GrpCmd{2} = repmat([zeros(1,5), 1],N,1);']);
else
    eval([varname,'.GrpCmd{1} = data.GrpCmd{1};']);
    eval([varname,'.GrpCmd{1} = data.GrpCmd{2};']);
end

if ~isfield(data,'Marker_T_W')
    eval([varname,'.Marker_T_W = [];']);
else
    eval([varname,'.Marker_T_W = data.Marker_T_W;']);
end

save([filepath,ActionName],varname)



          