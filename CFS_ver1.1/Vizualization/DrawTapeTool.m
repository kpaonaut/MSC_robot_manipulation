%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%       Visualize tapetool and create its handle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%  Parameters:
%  J62GrpPt: length from J6 to Gripper Finger Center.
%  GrpPt2ToolCenter: length from Gripper Finger Center to Tool Center.
%  robot_idx: define the tool grasping robot
% 
%  Note:
%  1. This function is to draw tape tool on the robot gripper, the  
%     The tapetool origin is attached to the robot tool frame.
%  2. If the taping tool is not on the robot, use 'DrawBlock' to draw.   
%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function handle = DrawTapeTool(Block, robot)

J62GrpPt = 0.2427;
GrpPt2ToolCenter = 0.09;

% robot_idx = 2;
if ~isfield(Block,'robot_idx') 
    warning('Taping Tool does not assigned on a robot, can not visualize the tool')
    return
else
    robot_idx = Block.robot_idx;
end

load('figure/block.mat');
i =3;  % block{3} is Taping Tool 

theta = deg2rad(Block.DesJntPos{robot_idx}(1,:));
M = getTransformM(theta,robot{robot_idx});

Ttcp = [eye(3), [0; 0; J62GrpPt + GrpPt2ToolCenter]; zeros(1,3) 1];   % from J6 to Tool
Mtcp = M{7}*Ttcp;
Rtcp = Mtcp(1:3,1:3);

% f=block{i}.f; v=block{i}.v+ones(size(block{i}.v,1),1)*Block.o(:,1)';  color=block{i}.color; %c=block{i}.c;
f=block{i}.f; v=block{i}.v;  color=block{i}.color; %c=block{i}.c;

for j=1:size(v,1)
    v(j,:)=v(j,:)*Rtcp' +Block.o(:,1)';
end

handle = patch('Faces',f,'Vertices',v,'FaceColor',color,'EdgeColor','None');
% handle = patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');

