%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%    Visualize the workspace and background setting 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Adjust the view by setting 'viewpoint' and 'zoom'
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function DrawWorkSpace(DrawOpt)
if ~isfield(DrawOpt, 'viewpoint'), DrawOpt.viewpoint = [1,-0.5,0.4]; end
if ~isfield(DrawOpt, 'zoom'), DrawOpt.zoom = 1; end

%% Draw Enviroment
figure(1); clf; hold on
% load('figure/stage.mat');
% load('figure/shortbench');
load('figure/longbench');
% for reduce stl file
patch('Faces',stage.f,'Vertices',stage.v,'FaceColor',stage.color,'EdgeColor','None');

% for original stl file
% patch('Faces',stage.f,'Vertices',stage.v,'FaceColor',stage.color,'EdgeColor','None');

%% Background Setting

axis equal
% for stage
% xlim = [-0.33, 1.4];
% ylim = [-1.2, 1]; %[-1.2, 0.6];
% zlim = [-0.343, 1.5];

% for long bench
xlim = [-0.33, 1.4];
ylim = [-2.3, 1]; %[-1.2, 0.6];
zlim = [-0.343, 1.5];

xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); 

axis([xlim, ylim, zlim])
view(DrawOpt.viewpoint)
zoom(DrawOpt.zoom)

% view([1,0.4,1])
% view([1,-0.5,0.4])

lighting flat
camlight('right');
% light=camlight('left');

wall{1}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(1),ylim(2),ylim(2),ylim(1)],[zlim(1),zlim(1),zlim(1),zlim(1)],[0.8 0.8 0.8]);
wall{2}.handle=fill3([xlim(1),xlim(1),xlim(1),xlim(1)],[ylim(1),ylim(1),ylim(2),ylim(2)],[zlim(1),zlim(2),zlim(2),zlim(1)],[0,0.9,0.9]);
wall{3}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(1),ylim(1),ylim(1),ylim(1)],[zlim(1),zlim(2),zlim(2),zlim(1)],[0,0.9,0.9]);

