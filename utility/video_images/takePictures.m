% Wenjie Chen, FANUC Corporation, 2016/04/19

%% Initial setup (path);
if exist('../init_setup.m', 'file')
    run('../init_setup.m');
end

%% Setup Kinect
PrintMsg('toolbox', 'Kinect2MATLAB');
k2 = Kin2('color', 'depth'); % , 'infrared', 'body'
PrintMsg('kinect ready');

%% Get Kinect data
opt.types = 'cd';   % color and depth outputs
opt.register = 1;
opt.datapath = '../data/picture/';
% opt.savedata = 1;
opt.plot_enb = 1;
opt.savefig = 1;
opt.colorScale = 1;
k2data = GetKinectData(k2, opt);

%% Close kinect object
k2.delete;
close all;

%% Image crop test
% imgFile = 'E:\Chin\Work\20151126_Skill_Learning\20151218_State_Mapping\data\picture\20160419_19062042_color.png';
% [img, numTargets, target_img] = getImgCrop(imgFile);