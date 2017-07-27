%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%    Generate transition submotion between two motion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%  Note:
%  This is a function to generate a transition sumotion between two motion,
%  where the generated transition motion will be added in the first motion.
%  If there is only one motion, no transition submotion is generated.
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function motion = TransMotionGeneration(motion,transtime, si, ri, gprcmd, opt)

totalmotion = length(motion);

for i = 1:totalmotion-1
    DesJntPos = [motion{i}.DesJntPos(end,:); motion{i+1}.DesJntPos(1,:)];
    if norm(DesJntPos(1,:) - DesJntPos(2,:)) > 1
        motion{i}.MoFlag(end+1) = 1;
    else
        motion{i}.Moflag(end+1) = 0;
    end
    motion{i}.DesJntPos(end+1,:) = motion{i+1}.DesJntPos(1,:);
    motion{i}.ReplayTime(end+1) = transtime;
    motion{i}.GrpCmd(end+1,:) = [zeros(1,5), 1];
    motion{i}.traj{end+1}.extcmd = TrajGenerate(DesJntPos,transtime,si,ri,gprcmd,opt);
end