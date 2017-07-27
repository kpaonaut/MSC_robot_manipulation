%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%   Main function to run the CFS path planning simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%  Based on Changliu's CFS algorithm 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%clear;

currfold = 'F:/TeTang/V4.0/CFS_ver1.1';
addpath([currfold,'/init']);
addpath([currfold,'/Lib']);
addpath([currfold,'/CFSplanner']);
addpath([currfold,'/DERIVESTsuite']);
addpath([currfold,'/distance']);
addpath([currfold,'/TrajGen']);
addpath([currfold,'/Motion']);
addpath([currfold,'/Vizualization']);

path = 'local'; % share

if strcmp(path,'local')
    % Local folder path 
    loadfilepath = [currfold,'\dataLTT\'];
    extcmdfilepath = [currfold,'\dataExtCmd\'];
    blkfilepath = [currfold,'\dataLTT\'];

elseif strcmp(path,'share')
    % Shared folder path
    loadfilepath = 'C:\Users\Hsien-Chung\Google Drive\FANUC_LTTData\beforeOpt\';
    extcmdfilepath = 'C:\Users\Hsien-Chung\Google Drive\FANUC_LTTData\afterOpt\';
    blkfilepath = 'C:\Users\Hsien-Chung\Google Drive\FANUC_LTTData\beforeOpt\';
    
else
    error('Plase specify local or share path')
end
% Previous file path

% ActionName = {'PressTapingTool_Ori'};

% filepath = [currfold,'\dataLTT\OrangeCable\'];%[currfold,'\dataLTT\MotionExample'];
% ActionName = {'WhiteNotch_04','BlackSlot_05','BlackSlot_03','BlackSlot_01'};%,'PressTapingTool'};

%% Initialization

% InitSim;
siSim = si;
riSim = si.ri;
InitDrawOpt;
robot = cell(1,2);
robot{1} = robotproperty('LRMate200iD7L');
robot{2} = robotproperty('LRMate200iD7Ld2');

robot{1}.bar = 'no';
robot{2}.bar = 'no';

opt.interpType = 'TCP';        % 'joint', 'TCP', 'linear'
opt.SI_jnt = 0;                % 0 - degree (default); 1 - radian
siSim.ext_vel_max_cnt = 10000; % original is 10000, Te use 15000

downsamplerate = 20;
transtime = 2;

%% Motion Selection

CaseName = 'UCBTest'; % 'OrangeLine'; %  'GreenLine'; 'PressTapingTool'
switch CaseName
    case 'OrangeLine'
        % WhiteNotch04 -> BlackSlot05  -> BlackSlot03  -> BlackSlot01 
        filename = 'OrangeLine';
        ActionName = {'WhiteNotch_04', 'BlackSlot_05', 'BlackSlot_03', 'BlackSlot_01'};
%         ActionName = { 'BlackSlot_03'};
    case 'GreenLine'
        % WhiteNotch02 -> BlackSlot01_End
        filename = 'GreenLine';      
        ActionName = {'WhiteNotch_02', 'BlackSlot_01_End'};
%           ActionName = {'BlackSlot_01_End'};
    case 'PressTapingTool'
        TapeCutfile = 'TapeCuttingApproach';
        filename = 'PressTapingTool';
        ActionName = {'PressTapingTool'};
        robot_idx = 1;                    % Assume Robot No.1 to press the button
        loadGeneralMotion(loadfilepath,TapeCutfile,filename,robot_idx);
    case 'UCBTest'
        filename = 'UCBtest';
        ActionName = {'UCBtest'};
end

%% Blocks
% Blockfilename = {'OrangeLine'};
% BlockList{1} = {'BlackSlot_03'};
Blockfilename = {'OrangeLine','GreenLine'};
BlockList{1} = {'WhiteNotch_04','BlackSlot_05','BlackSlot_03','BlackSlot_01'};
BlockList{2} = {'WhiteNotch_02'}; % 'BlackSlot_01_End' repeated

if strcmp(ActionName{1}(1:5),'Press')
    BlockList{3} = {'PressTapingTool'};
    Blockfilename{end+1} = 'PressTapingTool';
end

%% Generate Reference Motion
for r = 1:2
    for i = 1:size(ActionName,2)
        robot{r}.motion{i} = loadLTTMotion2(loadfilepath,filename, ActionName{i},r);
    end
end

tic;
for r =1:2
    for moid = 1:length(ActionName)
        robot{r}.motion{moid} = SubMotionGenerate(robot{r}.motion{moid}, siSim, riSim{r},[], opt);
    end
    robot{r}.motion =  TransMotionGeneration(robot{r}.motion, transtime, siSim, riSim{r},[], opt);
end

for r = 1:2
    for moid = 1:length(ActionName)
        robot{r}.motion{moid} = XrefGenerate(robot{r}.motion{moid},downsamplerate); 
    end
end

% "PressTapingTool" just uses start/end point to generate path, but we
% stii need the original path to do comparasion
% This is not fit for other motions!!
for moid = 1:length(ActionName)
    if strcmp(robot{1}.motion{1}.ActionName(1:5),'Press') %
        for r = 1:2
            robot{r} = ReplaceMotion(robot{r}, moid, siSim, riSim{r}, downsamplerate);
        end
    end
end
toc

%% Generate Obstacle Information
block = {}; 
for j = 1:length(Blockfilename)
    for i = 1:length(BlockList{j})
        block{end+1} = loadBlock2(blkfilepath,Blockfilename{j}, BlockList{j}{i});
        block{end} = buildBlkObs(block{end}, siSim);
        if strcmp(block{end}.name,'BlackSlot')
            transVec = [-0.05;0;0];            % AR Tag has -5cm offset in x direction in world frame
            block{end+1} = copyblock(block{end},transVec,siSim);
            block{end}.name = 'ARTag';
        end
    end
end

if strcmp(ActionName{1}(1:5),'Press')
    robot_idx = 2; % assume robot No.2 hold the tape tool
    [block{end}, toolbar ]= calculateTapeToolPos(robot,block{end}, robot_idx);
    block{end+1} = toolbar;
end

%% Concatenate Original Motion
orimotion{1} = [];
orimotion{2} = [];
mosteplist = zeros(length(ActionName),1);

for r = 1:2
    for j = 1:length(ActionName)
         [orimotion{r}, mosteplist(j)] = GenerateOrimotion(robot{r}.motion{j}, orimotion{r});
    end
    robot{r}.orimotion = orimotion{r};
end
robot{1}.mostep = mosteplist; % Use this to pause at each motion.

VizAll(robot, block, DrawOpt);

% %% Generate Moving Obstacle Motion (Human)
% % init_p = [1, 0, -0.5]';
% % init_R = rotz(5*pi/4);
% % T_human = [init_R,init_p;zeros(1,3),1];
% % [HuCap, ~] = HumanCap(T_human);
% % load('humanmotion');
% % HuMotionOriPos = [HuCap{3}.p(:,1); HuCap{3}.p(:,2); HuCap{4}.p(:,2)];
% % HuMotion = data1 - repmat(data1(:,1),1,size(data1,2))...
% %                  + repmat(HuMotionOriPos,1,size(data1,2));
% 
% %% Simulation
% % Currently, 'GreenLine' and 'OrangeLine' motion requires the tcp point
% % very close to the blocks, while PressTapingTool requires very large
% % margin to avoid collide ARtag and tool handle bar.
% if ~isfield(robot{1},'copymotion')  % Not Pressing Tool
%     robot{1}.margin = 0.01;
%     robot{2}.margin = 0.01;
%     robot{1}.cap{5}.r=0.01;
%     robot{2}.cap{5}.r=0.01;
% else                                % Pressing Taping Tool
%     robot{1}.margin = 0.03;
%     robot{2}.margin = 0.03;
%     robot{1}.cap{5}.r= 0.035;%0.035;
%     robot{2}.cap{5}.r= 0.035;%0.035;
% end
% 
% 
% newmotion{1} = [];
% newmotion{2} = [];
% dt = [];
% uref{1} = [];
% uref{2} = [];
% mosteplist = zeros(length(ActionName),1);
% 
% ComputeTime = [];
% Distance = [];
% blkidx = 1;
% for j = 1:length(ActionName)
%     SubMoStep = length(robot{1}.motion{j}.MoFlag);
%     
%     % Select the potential collision obstacles to optimize the path, when we
%     % do BlackSlot, we also need to consider "ARTag" as well.
%     blockset = {};
%     if ~strcmp(robot{1}.motion{j}.ActionName(1:5),'Black'),
%         blockset = block{blkidx};
%         blkidx = blkidx+1;
%     else
%         blockset = {block{blkidx},block{blkidx+1}};
%         blkidx = blkidx+2;
%     end
%     
%     for i = 1:SubMoStep 
%         id.MoID = j; 
%         id.SubMoID = i;       
%         status = MotionType(robot, id);
%         if ~strcmp(ActionName{1}(1:5),'Press') % Not Press Taping Tool
%             [xnew, xori, Data{i}] = CFSplanner(robot, id, blockset);
% %             [xnew, xori, Data{i}] = CFSplanner(robot, id, blockset, HuCap, HuMotion);
%             if isfield(Data{i},'time')
%                 ComputeTime = [ComputeTime; Data{i}.time];
%                 Distance = [Distance; Data{i}.dist];                
%             end
%         else                                   % Press Taping Tool
%             [xnew, xori, Data{i}] = CFSplanner(robot, id, block); % currently we consider all blocks
%             if ~isempty(Data{i})
%                 ComputeTime = [ComputeTime; Data{i}.time];
%                 Distance = [Distance; Data{i}.dist];
%             end
%         end
%         [newmotion, mostep]= GenerateNewmotion(robot, status, id, xnew, newmotion);
% %         [uref, ~] = GenerateUref(robot, status, id, Data{i}.uref, uref);
%         if isfield(Data{i},'dt')
%             dt = [dt; Data{i}.dt];
%         end
%     end
%     
%     mosteplist(j) = mostep; % this is for the pause option in VizAll
% 
% end
% 
% % Plot Temporal Optimization Result
% % plot_TimeOpt;
% 
% 
% %%
% % For Press Taping Tool, we want to compare the difference between user
% % predefine mid-points and CFS optmization only using start/end point.
% % If want to see the difference between before/after CFS, please comment
% % out the if-else condition.
% for r = 1:2
%     robot{r}.newmotion = newmotion{r};
%     if ~isfield(robot{r},'copymotion')
%         robot{r}.orimotion = orimotion{r};
%     else
%         copymotion = robot{r}.copymotion;
%         robot{r}.orimotion = GenerateOrimotion(copymotion);
%     end
% end
% robot{1}.mostep = mosteplist; % Use this to pause at each motion.
% 
% %% Generate ExtCmd
% for r = 1:2
%     ExtCmd{r} = Xref2ExtCmd(robot{r}.newmotion, robot{r}, siSim, riSim{r}, downsamplerate);
% %      If need to generate orimotion as extcmd, uncomment the line below
% %     OldCmd{r} = Xref2ExtCmd(robot{r}.orimotion, robot{r}, siSim, riSim{r}, downsamplerate);
% end
% ExtCmdfilename = ['ExtCmd-',filename];
% save([extcmdfilepath,ExtCmdfilename],'ExtCmd');
% 
% %% Visualization
% VizAll(robot, block, DrawOpt);
% 
% % Save Replay Motion so that you don't need to redo the optimization again
% ReplayMotion.name = filename;
% ReplayMotion.newmotion = newmotion;
% ReplayMotion.orimotion = orimotion;
% % ReplayMotion.mostep = mostep;
% ReplayMotion.block = block;
% save(['datareplay/Replay-',filename],'ReplayMotion');

