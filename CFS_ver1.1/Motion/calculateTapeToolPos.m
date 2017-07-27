%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%           build the tape tool properties
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%  Input:
%  robot   :  [1x1] struct, robot parameters
%  tapetool:  [1x1] struct, tape tool block generated from 'buildBlkObs' 
%  Output:
%  tapetool:  [1x1] struct, tape tool block
%  handlebar: [1x1] struct, handle bar block
%
%  Block Properties:
%  block.name: the name of the block
%  block.type: the shape definition, which determines the distance calculation method
%  block.p:    the characteristic points of the block
%  block.o:    the origin of the block CAD model (for visualization)
%  block.r:    the radius of the block
% 
%  Note:
%  1. The tapetool position is based on robot tool frame.
%  2. Please refer the "BlockSpec.pptx" for offset setting details
%  3. If the gripper is changed, update "J62Gripper" 
%  4. If tapetool changed, update "GrpPt2ToolCenter", "offset2ToolSide", 
%     "offset2bar1", and "offset2bar2". 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [tapetool, handlebar]= calculateTapeToolPos(robot,tapetool, robot_idx)

% Predifine Robot 2 hold the tape tool
% robot_idx = 2; 
J62GrpPt = 0.2427;
GrpPt2ToolCenter = 0.09;
offset2ToolSide = [0;0.01;0];
offset2bar1 = [-0.07;0;-0.04];
offset2bar2 = [-0.15;0;-0.09];

theta = deg2rad(tapetool.DesJntPos{robot_idx}(1,:));
M = getTransformM(theta,robot{robot_idx});

Ttool = eye(4);  Ttool(1:3,4) = [0;0;J62GrpPt + GrpPt2ToolCenter];
% Ttool = [eye(3), [0;0;J62GrpPt + GrpPt2ToolCenter]; zeros(1,3) 1];
Tp1 = Ttool; Tp2 = Ttool;
Tp1(1:3,4) = Tp1(1:3,4) + offset2ToolSide;
Tp2(1:3,4) = Tp2(1:3,4) - offset2ToolSide;
% Tp1 =  [eye(3), [0;0.01;0.2427 + 0.09]; zeros(1,3) 1];
% Tp2 =  [eye(3), [0;-0.01;0.2427 + 0.09]; zeros(1,3) 1];

Mtool = M{7}*Ttool;
Mp1 = M{7}*Tp1;
Mp2 = M{7}*Tp2;

toolp = Mtool(1:3,4);
p1 = Mp1(1:3,4);
p2 = Mp2(1:3,4);

tapetool.p = [p1,p2];
tapetool.o = toolp;
tapetool.robot_idx = robot_idx;


Tp3 = [eye(3), offset2bar1; zeros(1,3) 1];
Tp4 = [eye(3), offset2bar2; zeros(1,3) 1];
Mp3 = Mtool*Tp3;
Mp4 = Mtool*Tp4;

p3 = Mp3(1:3,4);
p4 = Mp4(1:3,4);

handlebar.name = 'ToolBar';
handlebar.type = 'cylinder';
handlebar.p = [p3,p4];
handlebar.r = 0.04;
handlebar.o = []; % no need to vizualization

