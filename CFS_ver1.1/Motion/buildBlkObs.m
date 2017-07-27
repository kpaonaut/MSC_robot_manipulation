%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%        build the block obstacle properties
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%  Input:
%  block: [1x1] struct, block parameters
%  si:    [1x1] struct, simulation environment parameters
% 
%  Output:
%  block: [1x1] struct, block parameters
%
%  Block Properties:
%  block.name       : the name of the block
%  block.Marker_T_W : the marker's transformation matrix in world frame
%  block.Block_T_DS : the block's transformation matrix in design space
%  block.DesJntPos  : the robot's position related to this motion (only valid for PressTapingTool now)
%  block.type       : the shape definition, which determines the distance calculation method
%  block.p          : the characteristic points of the block
%  block.o          : the origin of the block CAD model (for visualization)
%  block.r          : the radius of the block
% 
%  Note:
%  1. The block position is measured in world frame, but simulation is 
%     in design space. A coordinate transformation is required.
%  2. Please refer the "BlockSpec.pptx" for offset setting details
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function block = buildBlkObs(block, si)
BlockName = block.name;

% Marker_T_W is in world frame, but simulation is in Design Space.
T_DS2W = si.T_DS2W;
T_W2DS = [T_DS2W(1:3,1:3)', -T_DS2W(1:3,1:3)'*T_DS2W(1:3,4); zeros(1,3), 1];

if strcmp(BlockName(1:5),'Black')      % BlackGroove
    offset_marker2block = [0.05;0;0];  % offset from marker to block
    offset_t2o = [0; 0; -0.121];       % offset from top to CAD origin
    offset_o2b = [0;0; -0.051];        % offset from CAD origin to bottum
    block.name = 'BlackSlot';
    block.type = 'capsule';    
    Block_T_DS = T_W2DS*(block.Marker_T_W + [zeros(3),offset_marker2block;zeros(1,4)]);
    block.Block_T_DS = Block_T_DS;
    block.p = zeros(3,2);
    block.p(:,2) = Block_T_DS(1:3,4);
    block.p(:,1) = block.p(:,2) + offset_t2o + offset_o2b;
    block.o = block.p(:,2) + offset_t2o;
    block.r = 0.02;
    
elseif strcmp(BlockName(1:5),'White')  % WhiteSlot
    offset_t2o = [0; 0; -0.089];       % offset from top to CAD origin
    offset_o2b = [0;0; -0.037];        % offset from CAD origin to bottum
    block.name = 'WhiteNotch';
    block.type = 'cylinder';
    temp_T= [rotz(-pi/2),zeros(3,1);zeros(1,3), 1];  % currenty the marker coordinate has a pi/2 rotational offset in z axis
    Block_T_DS = T_W2DS*block.Marker_T_W*temp_T;
    block.Block_T_DS = Block_T_DS;
    block.p = zeros(3,2);
    block.p(:,2) = Block_T_DS(1:3,4);
    block.p(:,1) = block.p(:,2) + offset_t2o + offset_o2b;
    block.o = block.p(:,2) + offset_t2o;
    block.r = 0.044;
    
elseif strcmp(BlockName(1:5),'Press') % PressTapingTool
    % block position is defined by "calculateTapeToolPos"
    block.name = 'TapingTool';
    block.type = 'cylinder';
    block.Block_T_DS = [];
    block.p = [];
    block.r = 0.08;
    block.o = [];         
    
else
    block.name = [];
    block.type = [];
    block.Marker_T_W = [];
    block.Block_T_DS = [];
    block.p = [];
    block.r = [];
    block.o = [];
end

