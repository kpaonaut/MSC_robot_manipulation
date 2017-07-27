%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%   combine blocks and the static robot as a set of obstacles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%  Based on Changliu's CFS algorithm 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Input:
%  robot   : [1x2 cell], dual robot parameters
%  status  : [1x1 struct], identify the moving/static robot
%  block   : [1xN cell], N blocks property
%  id      : [1x1 struct], indicate the submotion
% 
%  Output:
%  obs    : [1x(N+M) cell], N blocks and M capsules on the static robot
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function obs = getObs(robot, status, block, id)
obs = {};

MoID = id.MoID;
SubMoID = id.SubMoID;

if strcmp(status.dualmotion,'no')
    StaticRobot = robot{status.StaticRobot};
    theta2 = StaticRobot.motion{MoID}.traj{SubMoID}.xref(1:StaticRobot.nlink);
    pos = CapPosMex(theta2, StaticRobot.DH, StaticRobot.base, StaticRobot.R_B2DS,StaticRobot.cap,StaticRobot.pos);
    if strcmp(StaticRobot.bar,'yes')
        obs{1} = pos{4}; obs{2} = pos{5}; obs{3} = pos{6}; 
        obs{1}.r = StaticRobot.cap{4}.r;
        obs{2}.r = StaticRobot.cap{5}.r;
        obs{3}.r = StaticRobot.cap{6}.r;
        obs{1}.type = 'capsule';
        obs{2}.type = 'capsule';
        obs{3}.type = 'capsule';
        obs{1}.ismove = 'no';
        obs{2}.ismove = 'no';
        obs{3}.ismove = 'no';
    else
        obs{1} = pos{4}; obs{2} = pos{5};
        obs{1}.r = StaticRobot.cap{4}.r; %0.03;
        obs{2}.r = StaticRobot.cap{5}.r; %0.01;
        obs{1}.type = 'capsule';
        obs{2}.type = 'capsule';
        obs{1}.ismove = 'no';
        obs{2}.ismove = 'no';
    end
end

if length(block)>1
    for i = 1:length(block)
        obs{end+1}.name = block{i}.name;
        obs{end}.type = block{i}.type;
        obs{end}.p = block{i}.p;
        obs{end}.r = block{i}.r;
        obs{end}.ismove = 'no';
    end
else
    if ~isempty(block.p)
        obs{end+1}.name = block.name;
        obs{end}.type = block.type;
        obs{end}.p = block.p;
        obs{end}.r = block.r;
        obs{end}.ismove = 'no';
    end
end
