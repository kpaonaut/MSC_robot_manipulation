%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor DataLog
%      Log Target PC Experiment Data to Host PC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin, 06/25/2015
%  MSC Lab, U.C.Berkeley
%  Modified by Wenjie Chen, 06/24/2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ExpData = DataLog(loadfile, opt)

if nargin < 2,  opt = [];  end
if ~isfield(opt,'relPath2Data') || isempty(opt.relPath2Data),  opt.relPath2Data = 'data\';  end   

n = numel(loadfile);

%% Load Data
% Attach to the target computer file system.
disp('-----> Connect to Target PC');
tg = SimulinkRealTime.target;
f  = tg.fs;

disp('-----> Downloading experimental data from Target PC ...');
for i = 1 : n
    %set the path
    fPath = sprintf('C:%s.DAT', loadfile{i});
    % Open the file, read the data, close the file.
    h = fopen(f, fPath);
    temp = fread(f, h);
    f.fclose(h);
    % Load the data
    x = SimulinkRealTime.utils.getFileScopeData(temp);
    v = matlab.lang.makeValidName(loadfile{i});
    eval([v '= x.data;']);
    clear temp
end
disp('-----> Experimental data loading completed.');

%% Write Experiment Data as Mat file
if ~isfield(opt,'fname_prefix') || isempty(opt.fname_prefix)
   opt.fname_prefix = 'ExpData_';
end
datatimestr = datestr(now,'yyyymmddHHMMSS');
save_fname = matlab.lang.makeValidName([opt.fname_prefix datatimestr]);
save([opt.relPath2Data, save_fname, '.mat'], loadfile{:});
ExpData = load([opt.relPath2Data, save_fname, '.mat']);
fprintf('-----> Data saved to Data folder.\n\n');

end
