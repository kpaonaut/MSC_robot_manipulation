% Wenjie Chen, 2016/06/13, FANUC Corporation
% This file is necessary to provide additional info for S-Function build
% when the source files are in different folders

function makeInfo=rtwmakecfg()
%RTWMAKECFG.m adds include and source directories to rtw make files.
%  makeInfo=RTWMAKECFG returns a structured array containing
%  following field:
%     makeInfo.includePath - cell array containing additional include
%                            directories. Those directories will be
%                            expanded into include instructions of Simulink
%                            Coder generated make files.
%
%     makeInfo.sourcePath  - cell array containing additional source
%                            directories. Those directories will be
%                            expanded into rules of Simulink Coder generated
%                            make files.

disp(['Running rtwmakecfg from folder: ',pwd]);
makeInfo.includePath = {fullfile(pwd, 'mex')};
makeInfo.sourcePath = {fullfile(pwd, 'mex')};
makeInfo.linkLibsObjs = {};

end