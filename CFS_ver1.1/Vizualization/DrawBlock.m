%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%       Visualize blocks and create its handle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Note:
%  1. The blocks position are measured in world frame, but the 
%     visualization is in design space. 
%  2. 'R_DS2W' needs to be updated if the world/design space frame
%     is changed.
%  3. If robot grasps the taping tool, use 'DrawTapeTool' to draw.   
%  4. To draw the block represent ARtag position, please uncomment
%     the corresponding code.
%  
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function handle = DrawBlock(Block)
load('figure/block.mat');

if strcmp(Block.name,'BlackSlot'), 
    i = 1;
elseif strcmp(Block.name,'WhiteNotch'),
    i = 2;     
elseif strcmp(Block.name,'TapingTool'), 
    i = 3;
% elseif strcmp(Block.name,'ARTag'),
%     i = 1; % temperarily to check collision with AR tags
else
    return;
end

% R_DS2W = [    0.7772    0.6292         0;
%              -0.6292    0.7772         0;
%                    0         0    1.0000]; 
R_DS = Block.Block_T_DS(1:3,1:3);               
% P_DS2W = [-0.2797, 0.5115, 0];

% f=block{i}.f; v=block{i}.v+ones(size(block{i}.v,1),1)*Block.o(:,1)';  color=block{i}.color; %c=block{i}.c;
f=block{i}.f; v=block{i}.v;  color=block{i}.color; %c=block{i}.c;
if ~isempty(Block.o) && i~= 3
    for j=1:size(v,1)
        v(j,:)=v(j,:)*R_DS' + Block.o(:,1)';
    end
end
if i == 3
    for j=1:size(v,1)
        v(j,:)=v(j,:) +Block.o(:,1)';
    end
end

handle = patch('Faces',f,'Vertices',v,'FaceColor',color,'EdgeColor','None');
% handle = patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
