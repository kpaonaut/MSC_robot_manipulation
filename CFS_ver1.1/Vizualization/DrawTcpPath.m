%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%       Visualize TCP path and create its handle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%  Parameters:
%  path: [3xN], the tcp points sets
% 
%  Note:
%  1. The default color is yellow, change the path color by adding
%     a second input to define the color
%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function handle = DrawTcpPath(path, varargin)
if isempty(varargin)
    color = 'y';
else
    color = varargin{1};
end
handle = plot3(path(1,:),path(2,:),path(3,:),'.-','LineWidth',3, 'Color',color);