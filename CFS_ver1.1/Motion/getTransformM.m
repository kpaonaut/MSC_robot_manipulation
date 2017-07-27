%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%   get robot transformation matrix for all joints in Design Space
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%  Input:
%  theta: [6x1] double, joint position
%  robot: [1x1] struct, robot parameters
% 
%  Output:
%  M: tranformation matrices, M{1} - base, M{2} - J1, ..., M{7} - J6
%     in Design Space.
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function M = getTransformM(theta,robot)
    M={};
    M{1}=[robot.R_B2DS robot.base;zeros(1,3) 1];
    DH=robot.DH;
    DH(:,1) = theta;
    DH(2,1) = DH(2,1) - pi/2; % offset in LR Mate 200iD7L Berkeley convention
    for i=1:6
        R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
            sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
            0  sin(DH(i,4)) cos(DH(i,4))];
        T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
        M{i+1}=M{i}*[R T;zeros(1,3) 1];
    end