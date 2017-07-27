%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%         Visualization Option Selection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  The option description is listed below.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% If yes, draw original path only (no need to run CFS)
% If no CFS result, also run original path, even this setting is "no"
DrawOpt.draworipath = 'no';

% Draw Robot CAD model
DrawOpt.drawrobot = 'yes';

% Draw Robot capsule
DrawOpt.drawcap = 'yes';

% Draw Robot Skeleton
DrawOpt.drawskeleton = 'no';

% Draw TCP path
DrawOpt.drawpath = 'yes';

% Add original path for comparasion
DrawOpt.addoripath = 'yes';

% Pause at each motion (not submotion)
DrawOpt.pause = 'yes';

% Delete capsule/skeleton at each time step
DrawOpt.delopt = 'yes';

% Setting capsule color
DrawOpt.color = ['r','b'];

% Setting capsule transparency
DrawOpt.valpha = 0.2;

% Setting Visualization playing speed
DrawOpt.FastStep = 2;

% Setting View Point
DrawOpt.viewpoint = [110,12] ; % x,y,z: [0.8,0.4,0.7]or az,el: [160,34] 

% Setting Zoom View
DrawOpt.zoom = 1.5; % 1,  2.5 , 3

% Obs Draw
DrawOpt.drawblock = 'yes'; % yes or no

% Motion plot
DrawOpt.motionplot = 'yes'; % yes or no

DrawOpt.drawhuman = 'no';


