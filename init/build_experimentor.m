%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       Build experimentor files
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/06/20
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function build_experimentor(opt)

fprintf('---> Now building experimentor files ...\n\n');

if nargin < 1,  opt = [];  end
if ~isfield(opt,'is_dual_arm') || isempty(opt.is_dual_arm),  opt.is_dual_arm = true;  end
if ~isfield(opt,'relPath2Experimentor') || isempty(opt.relPath2Experimentor),  opt.relPath2Experimentor = 'experimentor/';  end
if ~isfield(opt,'opensys') || isempty(opt.opensys),  opt.opensys = 0;  end

if opt.is_dual_arm
    slxMdlName = 'LRMate200Experimentor_Dual';
else
    slxMdlName = 'LRMate200Experimentor_Single';
end
check_robot_model_copy(opt);
if opt.opensys,  open_system(slxMdlName);  end

curFolder = pwd;
cd(opt.relPath2Experimentor);
rtwbuild(slxMdlName);
cd(curFolder);

end
