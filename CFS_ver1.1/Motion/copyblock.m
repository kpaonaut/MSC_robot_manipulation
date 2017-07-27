%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%          copy a block and move an offset
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%  Input:
%  block  : [1x1] struct, block parameters
%  trans_W: [3x1] double, a translation vector in "world frame"
%  si     : [1x1] struct, simulation environment parameters
% 
%  Output:
%  newblock:[1x1] struct, the copied block parameters
% 
%  Note:
%  1. trans_W is defined in world frame, but block is defined in Design 
%     space. Hence, a coordinate transformation is required.
%  2. This is originally used in generating AR tag block from BlackSlot
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function newblock = copyblock(block, trans_W, si)

T_DS2W = si.T_DS2W;
R_W2DS = T_DS2W(1:3,1:3)';
trans_DS = R_W2DS*trans_W;

newblock = block;
newblock.p = block.p + repmat(trans_DS,1,2);
newblock.o = block.o + trans_DS;