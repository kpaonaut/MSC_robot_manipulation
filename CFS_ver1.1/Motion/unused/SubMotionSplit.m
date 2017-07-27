function motion = SubMotionSplit(motion)
% Currently is not used. This function could be used to combine several
% small step into a submotion. 

N = size(motion.ReplayTime,1);
i = 1;

motion.traj = {};
SubMoFlag = [];
tempJntPos = motion.DesJntPos(1,:);%[];
statJntPos = motion.DesJntPos(1,:);%[];   
 
if length(motion.MoFlag) == 1
    motion.SubMoFlag = motion.MoFlag;
    motion.traj{1}.JntPos = motion.DesJntPos;
    motion.traj{1}.TimeVec = motion.ReplayTime;    
    return
end

if motion.MoFlag(i) == 1
    tempJntPos = [tempJntPos;motion.DesJntPos(i+1,:)];
    tempTimeVec = motion.ReplayTime(i);
    statJntPos =[];
else
    statJntPos =[statJntPos; motion.DesJntPos(i+1,:)];
    statTimeVec = motion.ReplayTime(i);
    tempTimeVec = [];
end
for i = 2:N
    if motion.MoFlag(i) == 1 , 
        if motion.MoFlag(i-1) == 0 
            AvgStaticJntPos = mean(statJntPos,1);
            motion.traj{end+1}.JntPos = repmat(AvgStaticJntPos,size(statTimeVec,1)+1,1);
            motion.traj{end}.TimeVec = statTimeVec;
            SubMoFlag = [SubMoFlag; 0];
            statJntPos = [];
            statTimeVec =[];
            
            tempJntPos = [AvgStaticJntPos; motion.DesJntPos(i+1,:)];
            tempTimeVec = motion.ReplayTime(i);
        else
            tempJntPos = [tempJntPos; motion.DesJntPos(i+1,:)];
            tempTimeVec = [tempTimeVec; motion.ReplayTime(i)];
        end
    else % motion.MoFlag(i) = 0
        if motion.MoFlag(i-1) == 1
            motion.traj{end+1}.JntPos = tempJntPos;
            motion.traj{end}.TimeVec = tempTimeVec;
            SubMoFlag = [SubMoFlag; 1];
            tempJntPos =[];
            tempTimeVec = [];
            
            statJntPos = motion.DesJntPos(i+1,:);
            statTimeVec = motion.ReplayTime(i);
        else
            statJntPos = [statJntPos; motion.DesJntPos(i+1,:)];
            statTimeVec = [statTimeVec;motion.ReplayTime(i)];
        end              
    end
    if i == N
        if motion.MoFlag(i) == 1, 
            motion.traj{end+1}.JntPos = tempJntPos;
            motion.traj{end}.TimeVec = tempTimeVec;
            SubMoFlag = [SubMoFlag; 1];
        else
           	AvgStaticJntPos = mean(statJntPos,1);
            motion.traj{end+1}.JntPos = repmat(AvgStaticJntPos,size(statTimeVec,1)+1,1);
            motion.traj{end}.TimeVec = statTimeVec;
            SubMoFlag = [SubMoFlag; 0];
        end
    end
end    
motion.SubMoFlag = SubMoFlag;