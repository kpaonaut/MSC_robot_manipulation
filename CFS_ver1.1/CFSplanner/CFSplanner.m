%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%        Convex Feasible Set Algorithm Solver
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%  Based on Changliu's CFS algorithm 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Input:
%  robot: [1x2 cell], dual robot parameters, if it is dual motion, robot{1}
%  contains the optimization parameters, otherwise, the moving robot
%  contains them.
%  id: [1x1 struct], indicates the robots motion
%  block: [1xN cell], blocks or tools in the workspace
%  
%  Output:
%  xnew: [horizon x Nstate], the optimized path. "horizon" is the
%  downsampled timestep, and "Nstate" is 6 or 12, which depends on dual 
%  motion or not.
%  xori: [horizon x Nstate], the original path, its dimension is the same 
%  as xnew.
%  data: [1x1 struct], store the CFS computation time and distance over the
%  horizon.
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [xnew, xori, data] = CFSplanner(robot, id, block, varargin)

%% Load Parameters
status = MotionType(robot, id);
TimeOpt = 'no'; % yes or no for Temporal Optimization
move_flag = false; % Moving Obstacles

if strcmp(status.move, 'no')
    status.dualmotion = 'yes';
    [xref, horizon] = getXref(robot,status, id);
    xnew = reshape(xref, length(xref)/horizon, horizon)';
    xori = xnew;   % no move, xori and xnew are the same
    data = [];
    if strcmp(TimeOpt,'yes')
        data.dt = ones(horizon,1)*robot{1}.delta_t; % If both robot no move, keep still for the task requirement
    end
    return
end
if strcmp(status.dualmotion,'no')
    % Define Move/Static Robot
    MovingRobot = robot{status.MovingRobot};
    StaticRobot = robot{status.StaticRobot};
end

if ~isempty(varargin)
    MovingObs = varargin{1};
    ObsMotion = varargin{2};
    move_flag = true;
end
%% Get Reference
[xref, horizon] = getXref(robot,status, id);

xori = reshape(xref, length(xref)/horizon, horizon);
xori = xori';

%% Specify Cost Fn & Constant Constraints
if strcmp(status.dualmotion,'no')
    MovingRobot = costFn_Dual(xref, horizon, MovingRobot); 
    MovingRobot = constFn_Dual(xref, horizon, MovingRobot);
else
    robot{1} = costFn_Dual(xref, horizon, robot{1}, robot{2});
    robot{1} = constFn_Dual(xref, horizon, robot{1}, robot{2});
end

% if you don't want to set the linear constraint, please comment out the
% code above, and uncomment the code below.
% for r = 1:2
%     robot{r}.opt.A = [];
%     robot{r}.opt.b = [];
%     robot{r}.opt.Aeq = [];
%     robot{r}.opt.beq = [];
%     robot{r}.opt.lb = [];
%     robot{r}.opt.ub = [];
% end

%% Get Obstacle Position
obs = getObs(robot, status, block, id);
if move_flag
    obs = getMovingObs(MovingObs,obs);
end
%% CFS Solver
if move_flag
    if strcmp(status.dualmotion,'no')
        [xnew,data] = CFS_solver_Dual(MovingRobot, StaticRobot, status, obs, xref, ObsMotion);
    else
        [xnew,data] = CFS_solver_Dual(robot{1}, robot{2}, status, obs, xref, ObsMotion);
    end
else
    if strcmp(status.dualmotion,'no')
        [xnew,data] = CFS_solver_Dual(MovingRobot, StaticRobot, status, obs, xref);
    else
        [xnew,data] = CFS_solver_Dual(robot{1}, robot{2}, status, obs, xref);
    end
end


if strcmp(TimeOpt, 'yes')
    if strcmp(status.dualmotion,'no')
        [znew, data2] = CFS_solver_TimeOpt2(MovingRobot, status, xnew);
        data.time = [sum(sum(data.time)), sum(data2.time)];
    else
        [znew, data2] = CFS_solver_TimeOpt2(robot{1}, status, xnew);
        data.time = [sum(sum(data.time)), sum(data2.time)];
    end        
    data.dt = znew(1:horizon);
    data.uref = reshape(znew(horizon+1:end), length(xnew)/horizon, horizon)';
    if max(abs(data.uref)) > 1.5
        max(abs(data.uref))
    end
end

xnew = reshape(xnew, length(xnew)/horizon, horizon);
xnew = xnew';



end