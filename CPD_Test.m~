%% Test Phase

%% Place the same object in the workspace, but in different location

%% Get Object Point Cloud at Test
k2 = cam_open('Kinect');
imThOpt = [];
imThOpt.plot_enb = 0;
imThOpt.downsample = 1;
cableData = cam2pcl(imThOpt, k2);
cam_close('Kinect', k2);

% denoise
ptCld = pcdenoise(cableData.ptCld, 'NumNeighbors', 10, 'Threshold', 2);

% Transform point cloud from camera coordinate to world coordinate
load('F:/TeTang/V4.0/sensing/T_W2K.mat');
xyz_K = ptCld.Location / 1000;   % change unit to m
xyz_W = (T_W2K(1:3,1:3)*xyz_K' + repmat(T_W2K(1:3,4),1,size(xyz_K,1)))';  % change unit to m
% define valid region
valid_idx = (xyz_W(:,1) > 0) & (xyz_W(:,1) < 1.5) & (xyz_W(:,2) > -0.7) & (xyz_W(:,2) < 0.7) & (xyz_W(:,3) > 0) & (xyz_W(:,3) < 0.3);
xyz_W = xyz_W(valid_idx,:);

% only take the x-y plane points
PtCld_Test = xyz_W(:,1:2);

%% Load Training Data (point cloud and LTT trajectory)
load('F:/TeTang/V4.0/data/Rope/Traj_Train_RopeKnot_1.mat') ;
load('F:/TeTang/V4.0/data/Rope/PtCld_Train_RopeKnot_1.mat') ;

%% Run CPD to Warp Traj for Robot 1 or 2
disp('Please double check which robot motion needs to be warped!');
WarpIndex = [1]; %[2], [1,2]
fig2_handle = figure(2); 
set(fig2_handle,'position', [962 562 958 434]);
rigidCompensate = 0;  % Choose whether to enable rotation compesation first
[LTT_Data_Test, warp] = LTT_Warping_CPD(LTT_Data_Train, PtCld_Train, PtCld_Test, si , '2D', WarpIndex, rigidCompensate);

% plot
fig3_handle = figure(3);
set(fig3_handle,'position', [962 562 958 434]);
orig_fig = subplot(1,2,1); scatter(PtCld_Train(:,1), PtCld_Train(:,2),'r*'); title('Train');
warp_fig = subplot(1,2,2); scatter(PtCld_Test(:,1), PtCld_Test(:,2),'r*'); title('Test');
draw_grid([-0.5 0.8], [1 -0.7], warp, 20, orig_fig, warp_fig)
subplot(orig_fig); axis equal;xlim([-0.5,1]); ylim([-0.7,0.8]); drawnow;
subplot(warp_fig); axis equal; xlim([-0.5,1]); ylim([-0.7,0.8]); drawnow;

%% Check collision if workpiece location is chaged substantially!
disp('======================================================================')
fig1_handle = figure(1);
set(fig1_handle,'position', [962 42 958 434]);

% save as LTT_Data_UCBtest for CFS check
LTT_Data_UCBtest = LTT_Data_Test;
save('F:/TeTang/V4.0/CFS_ver1.1/dataLTT/UCBTest.mat', 'LTT_Data_UCBtest');

% CFS check
CFS_Main
input('Use CFS toolbox to check collision. Press any key to execute the motion!!!')

cla(fig2_handle);
cla(orig_fig);
cla(warp_fig);
%% Download and Run Trajectory

% Brake Off
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(si.ParamSgnID, 'off');

% Rope Manipulation Movement
Traj_Download_Run(LTT_Data_Test, si, 'Download', 'Run');

% Brake On
wasBrakeOff = brake_on_off(si.ParamSgnID, 'on');
wasStopped = tg_start_stop('stop');
