%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       combine two motion primitives into a single traj 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Combined_Motion = combineMotion(First_Motion, Second_Motion, intervalTime)

Combined_Motion = {};
for i = 1:2
    Combined_Motion.DesJntPos{i} = [First_Motion.DesJntPos{i}; Second_Motion.DesJntPos{i}];
    Combined_Motion.GrpCmd{i} = [First_Motion.GrpCmd{i}; Second_Motion.GrpCmd{i}];
    Combined_Motion.ReplayTime{i} = [First_Motion.ReplayTime{i}; intervalTime; Second_Motion.ReplayTime{i}];
end
