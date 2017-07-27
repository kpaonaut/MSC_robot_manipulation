%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%    calculate the robot arm to obstacle/robot distance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%  Based on Changliu's CFS algorithm 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Input:
%  theta   : [Nstatex1], one or two robot joint position [rad]
%  obs     : [1xN cell], blocks or tools in the workspace
%  robot   : [1x1 cell], robot No.1/Moving Robot parameters
%  varagin : [1x1 cell], robot No.2 parameters (only valid in dualmotion)
% 
%  Output:
%  d : [1x1], minimum distance at current time step
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function d = CFSdistfun(theta,obs,robot,varargin)

%% Single Robot 
if isempty(varargin) %% Signle Robot
    nlink = size(robot.DH,1);
    d = Inf;
    pos = CapPosMex(theta, robot.DH, robot.base, robot.R_B2DS, robot.cap, robot.pos);
    
%%%%%%%%% Check robot to obstacle %%%%%%%%%%%     
for j = 1:size(obs,2)    
    % Check end effector first, if below a threshold, check whole body 
    dis0 = dist2obs(pos{5},obs{j});    
    dis0 = dis0 - robot.cap{5}.r;
    if dis0 < d,     d = dis0;      end
    
    if dis0 <= inf; %0.2
        for i=4:nlink
            dis = dist2obs(pos{i},obs{j});
            if dis < d,  d = dis;            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%% Dual Robot
else 
    nlink = size(robot.DH,1);
    d = Inf;
    
    theta1 = theta(1:nlink);
    theta2 = theta(nlink+1:2*nlink);
    
    robot2 = varargin{1};
    ri{1}.pos = CapPosMex(theta1, robot.DH, robot.base, robot.R_B2DS,robot.cap,robot.pos);
    ri{2}.pos = CapPosMex(theta2, robot2.DH, robot2.base, robot2.R_B2DS,robot2.cap,robot2.pos);
    
%%%%%%%%% Check robot to obstacle %%%%%%%%%%%
    for r = 1:2
        for j = 1:length(obs)
            
        % Check end effector first, if below a threshold, check whole body      
        dis0 = dist2obs(ri{r}.pos{5},obs{j});      
        dis0 = dis0 - robot.cap{5}.r;
        if dis0 < d,     d = dis0;      end
        
        if dis0 <= 0.1
            for i = 4:nlink 
                dis = dist2obs(ri{r}.pos{i},obs{j});
                dis = dis - robot.cap{i}.r;
                
                %     if norm(dis)<0.00001, dis = -norm(points(1:3)-ri{r}.pos{i}.p(:,2));  end
                if dis < d,     d = dis;      end
            end
        end
        end
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
%%%%%% Check dual robot collision %%%%%%%%%%%%
% only check link 4 - 6, ignore link 2 to save time
    if strcmp(robot.bar,'yes')
        linkID = [4,4; 4,5; 4,6; 5,4; 5,5; 5,6; 6,4; 6,5; 6,6];
    else
        linkID = [4,4; 4,5; 5,4; 5,5];
    end
    
    % Check end effector first, if below a threshold, check whole body
    [dis0, ~] = distLinSegMex(ri{1}.pos{5}.p(:,1),ri{1}.pos{5}.p(:,2),...
                              ri{2}.pos{5}.p(:,2),ri{2}.pos{5}.p(:,2));
    dis0 = dis0 - robot.cap{5}.r - robot2.cap{5}.r - robot.margin;
    if dis0 < d,     d = dis0;      end
    
    if dis0 <= 0.15
        for i = 1:size(linkID,1)
            [dis, ~] = distLinSegMex(ri{1}.pos{linkID(i,1)}.p(:,1),ri{1}.pos{linkID(i,1)}.p(:,2),...
                                     ri{2}.pos{linkID(i,2)}.p(:,2),ri{2}.pos{linkID(i,2)}.p(:,2));
            dis = dis - robot.cap{linkID(i,1)}.r - robot2.cap{linkID(i,2)}.r - robot.margin;
            if dis < d,     d = dis;      end                                 
        end
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
end