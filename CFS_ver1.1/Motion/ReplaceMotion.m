%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%  downsample extcmd and generate the vectorized path reference
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Note:
%  Currerntly this function is to replace "Press Taping Tool" for
%  comparasion.
%  DO NOT use it in other motions !!!
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function robot = ReplaceMotion(robot, idx, si, ri, dsrate)

oldmotion = robot.motion{idx};
robot.motion{idx} = [];
opt.interpType = 'tcp';

motion.ActionName = oldmotion.ActionName;
motion.DesJntPos = oldmotion.DesJntPos([1,4],:);
motion.ReplayTime = oldmotion.ReplayTime(2);
motion.GrpCmd = oldmotion.GrpCmd([1,4],:);
motion.Marker_T_W = oldmotion.Marker_T_W;
motion.MoFlag = oldmotion.MoFlag(2);
motion = SubMotionGenerate(motion, si, ri,[], opt);
motion = XrefGenerate(motion,dsrate); 

motion = SubMotionGenerate(motion, si, ri,[], opt);
motion = XrefGenerate(motion,dsrate); 

robot.copymotion = oldmotion;
robot.motion{idx} = motion;




