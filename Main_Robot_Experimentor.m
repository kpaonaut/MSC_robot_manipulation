%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Main File
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/06/20
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialization
addpath(genpath(fullfile(pwd, 'init')));
init_setup;
opt.is_dual_arm = true;

% setup robot experimentor parameters and then build
if opt.is_dual_arm
    si = init_LRMate200_Dual();
else
    si = init_LRMate200_Single();
end
build_experimentor(opt);

% set tunable parameters & go to initial positions
si.ParamSgnID = getParamSgnID(si);
init_pos_setup(si);

% open task interface (for high level task operations)
%main_gui;

% open Simulink Real Time Explorer (ususally only used for low level operations)
% slrtexplr;
