% Wenjie Chen, FANUC Corporation, 2016/01/15
% Te Tang Modified, 2017/04/12

clear; clc; close all;

filepath = 'F:\Common Tool\03Toolbox';

%% Check if the paths are already added
pathCell = regexp(path, pathsep, 'split');
Folder = [filepath,'\Kin2\Mex'];
if ispc  % Windows is not case-sensitive
    onPath = any(strcmpi(Folder, pathCell));
else
    onPath = any(strcmp(Folder, pathCell));
end

%% Setup path
if ~onPath
    
    % Experimentor model files
    addpath(fullfile(pwd, 'experimentor'));
    
    % Robotic toolbox, http://petercorke.com/Robotics_Toolbox.html
    run([filepath,'\rvctools\rvctools\startup_rvc.m']);
    
    % Shortcuts to run certain sequences of robot motions in target PC
    addpath(fullfile(pwd, 'moveit'));
    
    % robot basic library functions (Mex and C files for dynamics and inverse/forward kinematics)
    addpath(genpath(fullfile(pwd, 'rlib')));
    
    % Motion and force control related
    addpath(genpath(fullfile(pwd, 'motion')));
    
    % Safety check related
    addpath(genpath(fullfile(pwd, 'safety')));
    
    % Neural network related functions
    addpath(genpath(fullfile(pwd, 'nn')));
    
    % Image sensing related functions
    addpath(genpath(fullfile(pwd, 'sensing')));
    
    % Task planning related functions
    addpath(genpath(fullfile(pwd, 'task')));
    
    % Other utility functions
    addpath(genpath(fullfile(pwd, 'utility')));
    
    % Trajectory Warping related functions
    addpath(genpath(fullfile(pwd, 'trajwarp')));
    
    % http://www.mathworks.com/matlabcentral/fileexchange/53439-kinect-2-interface-for-matlab
    path([filepath,'\Kin2\Mex'], path);
    PrintMsg('toolbox', 'Kinect2MATLAB');
    
    % http://www.mathworks.com/matlabcentral/fileexchange/32226-recursive-directory-listing-enhanced-rdir
    path([filepath,'\Enhanced_rdir'], path);
    PrintMsg('toolbox', 'Enhanced_rdir');
    
    % setup point matching toolboxes (TPS-RPM, CPD)
    % https://www.cise.ufl.edu/~anand/students/chui/tps-rpm.html
%     path([filepath,'\TPS_RPM'], path);
%     PrintMsg('toolbox', 'TPS_RPM');
     
    % https://sites.google.com/site/myronenko/research/cpd
    cpdFolder = [filepath,'\CPD2'];
    addpath(genpath([cpdFolder, '/core']));
    addpath(genpath([cpdFolder, '/data']));
    addpath(genpath([cpdFolder, '/examples']));
    PrintMsg('toolbox', 'CPD');
    
    % Trajectory Warping related functions
    addpath(genpath([pwd, '/CFS_ver1.1']));
    PrintMsg('toolbox', 'CFS_ver1.1');
    % setup CNN toolbox (matconvnet)
    % http://www.vlfeat.org/matconvnet/
%     run([filepath,'\matconvnet\matconvnet\matlab\vl_setupnn.m']); %% CNN 
%     PrintMsg('toolbox', 'MatConvNet');
end

clear;
