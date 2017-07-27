%%
figure; hold on

load('figure/stage.mat');
patch('Faces',stage.f,'Vertices',stage.v,'FaceVertexCData',stage.c,'FaceColor',stage.color,'EdgeColor','None');


load(['figure/',ROBOT,'.mat']);

%% Draw Robot No.1
robot=robotproperty(ROBOT); robot.dualmotion = 'no';
offset= robot.base';

factor = 1;
% factor = 1/1000; % For M16iB 

%% Base
for i=1:1
    f=base{i}.f; v=base{i}.v*factor+ones(size(base{i}.v,1),1)*offset; c=base{i}.c; color=base{i}.color;
    patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
end

%% links
M={};
M{1}=[eye(3) offset';zeros(1,3) 1];
DH=robot.DH;
DH(:,1) = xref(1:njoint);
DH(2,1) = DH(2,1) - pi/2;
for i=1:6
    R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
        sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
        0  sin(DH(i,4)) cos(DH(i,4))];
    T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
    M{i+1}=M{i}*[R T;zeros(1,3) 1];
end
%%
for i=1:5
    v=link{i}.v*factor; f=link{i}.f; c=link{i}.c; color=link{i}.color;
    for j=1:size(v,1)
        v(j,:)=v(j,:)*M{i+1}(1:3,1:3)'+M{i+1}(1:3,4)';
    end
    patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
    
end

%% Payload
i=6;
v=payload{1}.v;f=payload{1}.f;c=payload{1}.c;color=payload{1}.color;
for j=1:size(v,1)
    v(j,:)=v(j,:)*M{i+1}(1:3,1:3)'+M{i+1}(1:3,4)';
end
patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');


%% Draw Robot No.2
robot2=robotproperty('LRMate200iD7Ld2');
offset= robot2.base';

factor = 1;
% factor = 1/1000; % For M16iB 

%% Base
for i=1:1
    f=base{i}.f; v=base{i}.v*factor+ones(size(base{i}.v,1),1)*offset; c=base{i}.c; color=base{i}.color;
    patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
end

%% links
M={};
M{1}=[robot2.R_B2DS offset';zeros(1,3) 1];
DH=robot2.DH;
if strcmp(robot.dualmotion,'yes')
    DH(:,1) = xref(njoint+1:njoint*2);
else
    DH(:,1) = traj{2}.xref(1:njoint); 
end
DH(2,1) = DH(2,1) - pi/2;


for i=1:6
    R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
        sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
        0  sin(DH(i,4)) cos(DH(i,4))];
    T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
    M{i+1}=M{i}*[R T;zeros(1,3) 1];
end
%%
for i=1:5
    v=link{i}.v*factor; f=link{i}.f; c=link{i}.c; color=link{i}.color;
    for j=1:size(v,1)
        v(j,:)=v(j,:)*M{i+1}(1:3,1:3)'+M{i+1}(1:3,4)';
    end
    patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
    
end

%% Payload
i=6;
v=payload{2}.v;f=payload{2}.f;c=payload{2}.c;color=payload{2}.color;
for j=1:size(v,1)
    v(j,:)=v(j,:)*M{i+1}(1:3,1:3)'+M{i+1}(1:3,4)';
end
patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');



%%
% axis equal
% % axis([-1 1,-1 1,0,2])
% view([1,0.4,1])
% %lighting flat
% light=camlight('right');
axis equal
xlim = [-0.33, 1.4];
ylim = [-1.2, 0.6];
zlim = [-0.4, 1.5];

axis([xlim, ylim, zlim])
% view([1,0.4,1])
view([1,-0.5,0.4])
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); 
lighting flat
light=camlight('left');


wall{1}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(1),ylim(2),ylim(2),ylim(1)],[zlim(1),zlim(1),zlim(1),zlim(1)],[0.8 0.8 0.8]);
wall{2}.handle=fill3([xlim(1),xlim(1),xlim(1),xlim(1)],[ylim(1),ylim(1),ylim(2),ylim(2)],[zlim(1),zlim(2),zlim(2),zlim(1)],[0,0.9,0.9]);
% wall{3}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(2),ylim(2),ylim(2),ylim(2)],[zlim(1),zlim(2),zlim(2),zlim(1)],[0,0.9,0.9]);
%%
load('figure/block.mat');
for i=1:1
    f=block{i}.f; v=block{i}.v*factor+ones(size(block{i}.v,1),1)*[0.45;0.24;0.29]'; c=block{i}.c; color=block{i}.color;
    patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
end