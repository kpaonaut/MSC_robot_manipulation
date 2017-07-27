%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%       set up the linear constant constraint
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%  Based on Changliu's CFS algorithm 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function robot = constFn_Dual(xref, horizon, robot, varargin)
% constaint constraint

%% jnt vel/acc bound
% thetadotmax = [r1J1; r2J1; r1J2; .... ]
if ~isempty(varargin) % Dual Robot
    robot2 = varargin{1};
    Nstate = robot.nlink + robot2.nlink;
    thetadotmax((1:length(robot.thetadotmax))*2-1) = robot.thetadotmax;
    thetadotmax((1:length(robot2.thetadotmax))*2) = robot2.thetadotmax;
else
    Nstate = robot.nlink;
    thetadotmax = robot.thetadotmax;
end

if size(thetadotmax,2)>1, thetadotmax =thetadotmax'; end


robot.opt.A = [robot.Vdiff;-robot.Vdiff];

vmax = repmat(thetadotmax,horizon,1);
vmin = repmat(thetadotmax,horizon,1);

% umax = repmat(robot.umax*ones(nstate,1),horizon,1);
% umin = -repmat(robot.umax*ones(nstate,1),horizon,1);
robot.opt.b = [10*vmax; 10*vmin];
% robot.opt.A = [];
% robot.opt.b = [];

%% initial state and final state
% robot.opt.Aeq = [eye(Nstate), zeros(Nstate,(horizon-1)*Nstate);
%                  zeros(Nstate,(horizon-1)*Nstate), eye(Nstate)];
% robot.opt.beq = [xref(1:Nstate); xref(end-Nstate+1:end)];

robot.opt.Aeq = [eye(Nstate), zeros(Nstate,(horizon-1)*Nstate);
                 eye(Nstate), -eye(Nstate), zeros(Nstate,(horizon-2)*Nstate);
                 zeros(Nstate,(horizon-1)*Nstate), eye(Nstate);
                 zeros(Nstate,(horizon-2)*Nstate), eye(Nstate), -eye(Nstate)];
robot.opt.beq = [xref(1:Nstate); zeros(Nstate,1); xref(end-Nstate+1:end); zeros(Nstate,1)];

% robot.opt.Aeq = [zeros(Nstate,(horizon-1)*Nstate), eye(Nstate)];
% robot.opt.beq = xref(end-Nstate+1:end);

% robot.opt.Aeq = [];
% robot.opt.beq = [];

%% joint limit
if ~isempty(varargin) % Dual Robot
robot.opt.lb = repmat([robot.thetamax(:,1);robot2.thetamax(:,1)],horizon,1);
robot.opt.ub = repmat([robot.thetamax(:,2);robot2.thetamax(:,2)],horizon,1);
else
    robot.opt.lb = repmat(robot.thetamax(:,1),horizon,1);
    robot.opt.ub = repmat(robot.thetamax(:,2),horizon,1);
end