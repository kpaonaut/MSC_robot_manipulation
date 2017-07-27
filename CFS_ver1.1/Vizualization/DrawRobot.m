%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%    Visualize single robot and create its handle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function handle = DrawRobot(robot, M, robotCAD)

base = robotCAD.base;
link = robotCAD.link;
payload = robotCAD.payload;

if ~isfield(robot,'bar'), robot.bar ='no'; end
%% Draw Robot No.1
offset= robot.base';
factor = 1;
% factor = 1/1000; % For M16iB 

%% Base
for i=1:1
    f=base{i}.f; v=base{i}.v*factor+ones(size(base{i}.v,1),1)*offset; color=base{i}.color;
%     c=base{i}.c; 
    patch('Faces',f,'Vertices',v,'FaceColor',color,'EdgeColor','None');
%     patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
%     for original stl file
end

%% 


%% links
for i=1:5
    v = link{i}.v*factor; f=link{i}.f;  color=link{i}.color; %c=link{i}.c;
    for j=1:size(v,1)
        v(j,:)=v(j,:)*M{i+1}(1:3,1:3)'+M{i+1}(1:3,4)';
    end
    handle(i) = patch('Faces',f,'Vertices',v,'FaceColor',color,'EdgeColor','None');
%     patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
%     for original stl file
end
%% Payload
i=6;
if strcmp(robot.bar,'no')
v = payload{1}.v;f=payload{1}.f;color=payload{1}.color; %c=payload{1}.c;
else
v = payload{2}.v;f=payload{2}.f;color=payload{2}.color; %c=payload{1}.c;
end

for j=1:size(v,1)
    v(j,:)=v(j,:)*M{i+1}(1:3,1:3)'+M{i+1}(1:3,4)';
end
handle(i) = patch('Faces',f,'Vertices',v,'FaceColor',color,'EdgeColor','None');
%     patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
%     for original stl file
