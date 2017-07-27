%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%         get static robot initial position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function StaticJntPos = getStaticPos(robot, status, id)
MoID = id.MoID;
SubMoID = id.SubMoID;
StaticRobot = robot{status.StaticRobot};
StaticJntPos = StaticRobot.motion{MoID}.traj{SubMoID}.xref(1:StaticRobot.nlink);