%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%       Visualize the whole simulation result
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  See InitDrawOpt to know further setting details
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function VizAll(robot,  block , DrawOpt, varargin)
nstate = robot{1}.nlink; 

HuMotion = [];
if ~isempty(varargin)
    HuMotion = varargin{1};
    DrawOpt.drawhuman = 'yes';
end

Xori = [robot{1}.orimotion, robot{2}.orimotion]';

if strcmp(DrawOpt.draworipath,'yes') || ~isfield(robot{1},'newmotion')
    Xnew = Xori; % To Draw Origin Path
else
    Xnew = [robot{1}.newmotion, robot{2}.newmotion]'; % To Draw New Path
end

horizon = size(Xnew,2);

valpha = DrawOpt.valpha;
FastStep =  DrawOpt.FastStep; % To speed up the Visualization

m = 1; % index for pause
%% Draw Environment
DrawWorkSpace(DrawOpt);

%% Draw Dual Robot
robotCAD = load(['figure/',robot{1}.name,'.mat']);
clear handle

% Initialize
theta = zeros(6,2);

theta(:,1) = Xnew(1:nstate,1);
theta(:,2) = Xnew(nstate+1:nstate*2,1);

if strcmp(DrawOpt.drawpath,'yes')
    TcpPath{1} = [];
    TcpPath{2} = [];
end

if strcmp(DrawOpt.addoripath,'yes')
%     Xori = reshape(xori,Nstate, length(xori)/Nstate);
    OriPath{1} = [];
    OriPath{2} = [];
end

M1 = getTransformM(theta(:,1),robot{1});
M2 = getTransformM(theta(:,2),robot{2});

if strcmp(DrawOpt.drawrobot, 'yes')
    handle.robot{1} = DrawRobot(robot{1},M1, robotCAD);
    handle.robot{2} = DrawRobot(robot{2},M2, robotCAD); 
end

if strcmp(DrawOpt.drawskeleton,'yes')
    handle.skeleton{1} = DrawSkeleton(robot{1},M1,DrawOpt.color(1));
    handle.skeleton{2} = DrawSkeleton(robot{2},M2,DrawOpt.color(2));
end



pause(0.01);
%% Draw Human
if strcmp(DrawOpt.drawhuman,'yes')
    opt.plotleg = 'yes';

    init_p = [1, 0, -0.5]';
    init_R = rotz(5*pi/4);
    T_human = [init_R,init_p;zeros(1,3),1];
    HuCap = HumanCap(T_human);

%     HuCap = UpdateHuCap(HuCap, HuMotion(:,1));

    handle.human = DrawHuman(HuCap, 'r', opt);
end

%% Draw Block
if strcmp(DrawOpt.drawblock,'yes')
    for i =1:length(block)
        if ~strcmp(block{i}.name,'TapingTool')
            DrawBlock(block{i});
        else
            DrawTapeTool(block{i},robot);
        end
    end  
end
pause(0.01);
if strcmp(DrawOpt.motionplot,'yes')
%% Draw Origin Path
if strcmp(DrawOpt.addoripath,'yes')
    for i = 1:length(Xori)
        theta1 = Xori(1:nstate,i); theta2 = Xori(nstate+1:nstate*2,i);
        M1 = getTransformM(theta1,robot{1});
        M2 = getTransformM(theta2,robot{2});
        OriPath{1} = getTcpPath(OriPath{1},M1);
        OriPath{2} = getTcpPath(OriPath{2},M2);
    end
    handle.OriPath{1} = DrawTcpPath(OriPath{1},'c');
    handle.OriPath{2} = DrawTcpPath(OriPath{2},'c');
    pause(0.01);
end

%% Visualize the result
if strcmp(DrawOpt.drawhuman,'yes')
    delete(handle.human);
end
for i=1:FastStep:horizon
    
    theta1 = Xnew(1:nstate,i);
    theta2 = Xnew(nstate+1:end,i);  

    M1 = getTransformM(theta1,robot{1});
    M2 = getTransformM(theta2,robot{2});

% Draw Robot
if strcmp(DrawOpt.drawrobot, 'yes')
    handle.robot{1} = UpdateRobot(handle.robot{1},M1,robotCAD);
    handle.robot{2} = UpdateRobot(handle.robot{2},M2,robotCAD);
end
% Draw RobotCap;
if strcmp(DrawOpt.drawcap, 'yes')
    handle.cap{1} = DrawCapLink(robot{1}, M1, DrawOpt.color(1), valpha);
    handle.cap{2} = DrawCapLink(robot{2}, M2, DrawOpt.color(2), valpha);
end
% Draw Skeleton
if strcmp(DrawOpt.drawskeleton,'yes')
    handle.skeleton{1} = DrawSkeleton(robot{1},M1,DrawOpt.color(1));
    handle.skeleton{2} = DrawSkeleton(robot{2},M2,DrawOpt.color(2));
end


% Draw Human
if strcmp(DrawOpt.drawhuman,'yes')
%     if isempty(HuMotion)
%         HuCap = HumanCap();
%     else
        HuCap = UpdateHuCap(HuCap, HuMotion(:,i));
%     end
    handle.human = DrawHuman(HuCap, 'r', opt);
end

% Draw TCP Path
if strcmp(DrawOpt.drawpath, 'yes')
    TcpPath{1} = getTcpPath(TcpPath{1},M1);
    TcpPath{2} = getTcpPath(TcpPath{2},M2);
    handle.path{1} = DrawTcpPath(TcpPath{1},'y');
    handle.path{2} = DrawTcpPath(TcpPath{2},'y');
end

% Pause 
pause(0.001);
if strcmp(DrawOpt.pause,'yes')
    if i > robot{1}.mostep(m), pause; m = m+1; end
end

% Delete handle
if strcmp(DrawOpt.delopt, 'yes')
    if i ~= horizon
        if strcmp(DrawOpt.drawcap, 'yes'),    
            delete(handle.cap{1});
            delete(handle.cap{2}); 
        end
        if strcmp(DrawOpt.drawskeleton, 'yes'), 
            delete(handle.skeleton{1});
            delete(handle.skeleton{2});
        end
        if strcmp(DrawOpt.drawhuman, 'yes'), 
            delete(handle.human);
        end
    end
end
end
end