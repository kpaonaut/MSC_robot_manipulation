%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%    Visualize single robot capsule and create its handle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  For wire hanress application, we didn't draw capsule on the gripper,
%  If want to draw the capsule on the gripper, use n = size(M,2) -1;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function handle = DrawCapLink(robot, M, color, valpha)

if ~isfield(robot,'bar'), robot.bar ='no'; end
boundary=robot.boundary;
if strcmp(robot.bar,'yes')
    n=size(M,2);
else
    n = size(M,2)-2; % not draw gripper part
end
for i=1:n %2:n-1
    if size(boundary{i},1)>0
        X=boundary{i}.X;
        Y=boundary{i}.Y;
        Z=boundary{i}.Z;
        kd=size(X,1);jd=size(X,2);
        for k=1:kd
            for j=1:jd
                newvec=[X(k,j),Y(k,j),Z(k,j)]*M{i}(1:3,1:3)'+M{i}(1:3,4)';
                X(k,j)=newvec(1);
                Y(k,j)=newvec(2);
                Z(k,j)=newvec(3);
            end
        end
        handle(i)=surf(X,Y,Z,'FaceColor',color,'EdgeColor','None');
        alpha(handle(i),valpha);
    end
end