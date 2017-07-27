%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%  Return which robot is moving in the indicated submotion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function Status = MotionType(robot,id)
MoID = id.MoID;
SubMoID = id.SubMoID;
SubMoFlag = [robot{1}.motion{MoID}.MoFlag(SubMoID), robot{2}.motion{MoID}.MoFlag(SubMoID)];
% SubMoFlag = [robot{1}.motion{MoID}.SubMoFlag(SubMoID), robot{2}.motion{MoID}.SubMoFlag(SubMoID)];

if sum(SubMoFlag) >= 2, 
    Status.move = 'yes';
    Status.dualmotion = 'yes';
elseif sum(SubMoFlag) == 1
    Status.move = 'yes';
    Status.dualmotion = 'no';    
else
    Status.move = 'no';
    Status.dualmotion = 'no';
end
Status.MovingRobot = find(SubMoFlag==1);
Status.StaticRobot = find(SubMoFlag==0);