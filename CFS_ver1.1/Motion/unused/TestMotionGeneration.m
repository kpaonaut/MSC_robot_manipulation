currfold = pwd;
addpath([currfold,'/Lib']);
addpath([currfold,'/Lib_Dual']);
addpath([currfold,'/DERIVESTsuite']);
addpath([currfold,'/TrajGen']);
addpath([currfold,'/Motion']);
addpath([currfold,'/Vizualization']);

InitSim;


filepath = [currfold,'\dataLTT\'];%[currfold,'\dataLTT\MotionExample'];
ActionName = {'WhiteSlot','BlackGroove'};%,'PressTapingTool'};
robot = cell(1,2);
robot{1} = robotproperty('LRMate200iD7L');
robot{2} = robotproperty('LRMate200iD7Ld2');

for r = 1:2
    for i = 1:size(ActionName,2)
        robot{r}.motion{i} = loadLTTMotion(filepath,ActionName{i},r);
    end
end


robot{1}.bar = 'no';
robot{2}.bar = 'no';
opt.interpType = 'tcp';  % 'joint', 'TCP', 'linear'
opt.SI_jnt = 0;          % 0 - degree (default); 1 - radian
downsamplerate = 20;


for r =1:2
    for moid = 1:2
%         robot{r}.motion{moid} = SubMotionSplit(robot{r}.motion{moid});
%         % check, but it is not necessary to use
        robot{r}.motion{moid} = SubMotionGenerate(robot{r}.motion{moid}, si, ri{r},[], opt); % check
        robot{r}.motion{moid} = XrefGenerate(robot{r}.motion{moid},downsamplerate);          % check
    end
end

%%

id.MoID =2; 
id.SubMoID = 2; 


status = MotionType(robot, id);                         % check

[xref, xori, horizon] = getXref(robot, status, id);     % check

ObsPos{1}.p = [[0.45;0.24;0.29] [0.45;0.24;0.42]];
ObsPos{2}.p = [0;0;0.25];

obs = getObs(robot,status,ObsPos,id);                   % check

MovingRobot = robot{status.MovingRobot};
StaticRobot = robot{status.StaticRobot};

MovingRobot = costFn_Dual(xref, horizon, MovingRobot); % check
MovingRobot = constFn_Dual(xref, horizon, MovingRobot);% some problem cause QP infeasible

robot{1}.margin = 0.03;
robot{2}.margin = 0.03;
[xnew1, xori1, dd] = CFSplanner(robot, id);               % check
%%
Viz(robot,status,id, xnew1, xori1);
%%
% Xnew1 = reshape(xnew1,robot{1}.nlink,length(xnew1)/robot{1}.nlink);
% Vnew1 = diff(Xnew1);
% figure; plot(Vnew1', 'Marker','.');
% figure; plot(Xnew1', 'Marker','.');

ExtCmd = Xref2ExtCmd(xnew1, robot{status.MovingRobot}, si, ri{1}, downsamplerate);