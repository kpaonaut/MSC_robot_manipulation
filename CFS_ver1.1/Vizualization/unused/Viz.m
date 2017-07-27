function Viz(robot,status,id, xnew,xori)
nstate = robot{1}.nlink;
if strcmp(status.dualmotion,'yes'), 
    Nstate = nstate*2;
else
    Nstate = nstate;
end
% Xnew = reshape(xnew,Nstate,length(xnew)/Nstate);
Xnew = xnew'; 
Xori = xori';

horizon = size(Xnew,2);
DrawOpt.drawcap = 'yes';
DrawOpt.drawrobot = 'yes';
DrawOpt.drawskeleton = 'no';
DrawOpt.drawpath = 'yes';
DrawOpt.draworipath = 'yes';

DrawOpt.pause = 'no';
DrawOpt.fade = 'no';
DrawOpt.delopt = 'yes';
DrawOpt.color = ['r','b'];
DrawOpt.valpha = 0.2;

valpha = DrawOpt.valpha;

%% Draw Environment
DrawWorkSpace

%% Draw Dual Robot
robotCAD = load(['figure/',robot{1}.name,'.mat']);
clear handle
% Initialize
theta = zeros(6,2);
nlink = 6;
if strcmp(status.dualmotion,'yes')
    theta(:,1) = xnew(1:nlink);
    theta(:,2) = xnew(nlink+1:nlink*2);
else
    theta(:,status.MovingRobot) = xnew(1:nlink);
    theta(:,status.StaticRobot) = getStaticPos(robot,status,id);
end

if strcmp(DrawOpt.drawpath,'yes')
    TcpPath{1} = [];
    TcpPath{2} = [];
end

if strcmp(DrawOpt.draworipath,'yes')
%     Xori = reshape(xori,Nstate, length(xori)/Nstate);
    OriPath{1} = [];
    OriPath{2} = [];
end

M1 = getTransformM(theta(:,1),robot{1});
M2 = getTransformM(theta(:,2),robot{2});

handle.robot{1} = DrawRobot(robot{1},M1, robotCAD);
handle.robot{2} = DrawRobot(robot{2},M2, robotCAD);


pause;
if strcmp(status.dualmotion,'yes')
    if strcmp(DrawOpt.draworipath,'yes')
        for i = 1:horizon
            theta1 = Xori(:,i); theta2 = Xori(:,2);
            M1 = getTransformM(theta1,robot{1});
            M2 = getTransformM(theta2,robot{2});
            OriPath{1} = getTcpPath(OriPath{1},M1);
            OriPath{2} = getTcpPath(OriPath{2},M2);
        end
        handle.OriPath{1} = DrawTcpPath(OriPath{1},'c');
        handle.OriPath{2} = DrawTcpPath(OriPath{2},'c');
        pause;
    end
    for i=1:1:horizon
        % new trajectory
        theta1 = Xnew(1:nstate,i);
        theta2 = Xnew(nstate+1:end,i);
        % origin trajectroy
    %     theta1 = xori1(:,i);
    %     theta2 = xori2(:,i);    

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
            handle.skeleton{1} = DrawSkeleton(robot{2},M2,DrawOpt.color(2));
        end
        % Draw TCP Path
        if strcmp(DrawOpt.drawpath, 'yes')
            TcpPath{1} = getTcpPath(TcpPath{1},M1);
            TcpPath{2} = getTcpPath(TcpPath{2},M2);
            handle.path{1} = DrawTcpPath(TcpPath{1});
            handle.path{2} = DrawTcpPath(TcpPath{2});
        end  
    end
else
    % assign moving / static robot
    idm = status.MovingRobot; ids = status.StaticRobot;
    color1 = DrawOpt.color(idm);
    color2 = DrawOpt.color(ids);
   
    % Draw Static Robot CapLink
    Mstatic = getTransformM(theta(:,status.StaticRobot), robot{ids});
    handle.cap{ids} = DrawCapLink(robot{ids}, Mstatic, color2, valpha);

    if strcmp(DrawOpt.draworipath,'yes')
        for i = 1:horizon
            theta1 = Xori(:,i); theta2 = Xori(:,2);
            M = getTransformM(theta1,robot{idm});
            OriPath{idm} = getTcpPath(OriPath{idm},M);            
        end
        handle.OriPath{idm} = DrawTcpPath(OriPath{idm},'c');
        pause;
    end    
     
    for i = 1:horizon
        theta1 = Xnew(1:nstate,i);
        M = getTransformM(theta1,robot{idm});
    % Draw Robot
    if strcmp(DrawOpt.drawrobot, 'yes')
        handle.robot{idm} = UpdateRobot(handle.robot{idm},M,robotCAD);
    end
    % Draw RobotCap;
    if strcmp(DrawOpt.drawcap, 'yes')
        handle.cap{idm} = DrawCapLink(robot{idm}, M, color1, valpha);
    end
    % Draw Skeleton
    if strcmp(DrawOpt.drawskeleton,'yes')
        handle.skeleton{idm} = DrawSkeleton(robot{idm},M,color1);
    end
    % Draw TCP Path
    if strcmp(DrawOpt.drawpath, 'yes')
        TcpPath{idm} = getTcpPath(TcpPath{idm},M);
        handle.path{idm} = DrawTcpPath(TcpPath{idm});
    end
    
    
    pause(0.01);
    if strcmp(DrawOpt.pause,'yes')
        if mod(i,5)==0, pause; end
    end
    
    % Delete handle
    if strcmp(DrawOpt.delopt, 'yes')
        if i ~= horizon
            if strcmp(DrawOpt.drawcap, 'yes'),    delete(handle.cap{idm});      end
            if strcmp(DrawOpt.drawskeleton, 'yes'), delete(handle.skeleton{idm}); end
        end
    end
    end    
end