%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%    Visualize robot skeleton and create its handle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function handle = DrawSkeleton(robot, M, color)

if ~isfield(robot,'bar'), robot.bar ='no'; end

if strcmp(robot.bar,'yes')
    n=size(robot.cap,2);
else
    n = size(robot.cap,2)-1;
end

for i=1:robot.nlink
    for k=1:2
        pos{i}.p(:,k)=M{i+1}(1:3,1:3)*robot.cap{i}.p(:,k)+M{i+1}(1:3,4);
    end
end
for i=1:n
    if robot.cap{i}.r>0
        handle(i) = plot3(pos{i}.p(1,:),pos{i}.p(2,:),pos{i}.p(3,:),'.-','LineWidth',10,'MarkerSize',50, 'Color',color);
    end
end