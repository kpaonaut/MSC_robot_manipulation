%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%   calculate the robot tcp point, and concatenate the points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%  When Gripper is changed, it is necessary to update "J62GrpPt"
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function path = getTcpPath(path, M)

% DH for tcp [0, 0.2427, 0 ,0]
J62GrpPt = 0.2427;
Ttcp = [eye(3), [0;0;J62GrpPt]; zeros(1,3) 1];
Mtcp = M{7}*Ttcp;
TCP = Mtcp(1:3,4);
path = [path, TCP];