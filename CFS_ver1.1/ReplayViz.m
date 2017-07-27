%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%          Replay the visualization result
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Load the "Replay-ActionName", and visualize the simulation
%  Replay-ActiomnName files are save in folder "replay" 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

currfold = pwd;
addpath([currfold,'/Lib']);
addpath([currfold,'/Lib_Dual']);
addpath([currfold,'/Vizualization']);
InitDrawOpt;
ReplayActionName = 'GreenLine'; %'PressTapingTool';% %'OrangeLine';

Replayrobot = cell(1,2);
Replayrobot{1} = robotproperty('LRMate200iD7L');
Replayrobot{2} = robotproperty('LRMate200iD7Ld2');


load(['datareplay\Replay-',ReplayActionName]);

for r = 1:2
    Replayrobot{r}.newmotion = ReplayMotion.newmotion{r};
    Replayrobot{r}.orimotion = ReplayMotion.orimotion{r};
    Replayrobot{r}.mostep = ReplayMotion.mostep;
end

VizAll(Replayrobot, ReplayMotion.block, DrawOpt)