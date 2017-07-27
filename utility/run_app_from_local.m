%% Wenjie Chen, FANUC Corporation, 2016/08/04
% Run the windows external program from its local path

function localPath = run_app_from_local(fName, opt)

if nargin < 2,  opt = '';  end

curPath = pwd;
fnLen = length(fName);
localPath = which(fName);
localPath = localPath(1:end-fnLen);
cd(localPath);
eval(['!', fName, opt]);
cd(curPath);

end
