%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function updates the axis of the capsules
%
% Origin from Changliu Liu
% Modified by Hsien-Chung Lin 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [pos,M]=CapPos_Dual(theta, robot)

base = robot.base;
DH = robot.DH;
RoCap = robot.cap;
nstate = size(DH,1);

for i=1:nstate
    DH(i,1)=theta(i);
end%theta,d,a,alpha
DH(2,1) = DH(2,1) - pi/2; % Offset in the LRMate200iD7L

if isempty(robot.R_B2DS), 
    R_B2DS = eye(3);
else
    R_B2DS = robot.R_B2DS;
end;

if size(base,2)>1,    base=base';    end

nlink=size(DH,1);
pos=cell(1,nlink);

M=cell(1,nlink+1); M{1}=[R_B2DS, base;zeros(1,3) 1];
for i=1:nlink
    R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
        sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
        0  sin(DH(i,4)) cos(DH(i,4))];
    T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
    M{i+1}=M{i}*[R T; zeros(1,3) 1];
    for k=1:2
        pos{i}.p(:,k)=M{i+1}(1:3,1:3)*RoCap{i}.p(:,k)+M{i+1}(1:3,4);
    end
end
end