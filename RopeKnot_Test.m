%% Rope Knotting Demo
% Run 4 steps sequentially

% Brake Off
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(si.ParamSgnID, 'off');

for RopeStep = 1:4


clear LTT_Data_Train LTT_Data_Test PtCld_Train  PtCld_Test WarpIndex
load(['F:/TeTang/V4.0/data/Rope/Traj_Train_RopeKnot_',num2str(RopeStep),'.mat']) ;
load(['F:/TeTang/V4.0/data/Rope/PtCld_Train_RopeKnot_',num2str(RopeStep),'.mat']) ;

% For different step, different robot arm needs to be warped. (one robot moves, another robot remains steady)
switch RopeStep
    case 1
        WarpIndex = 1; % mv robot No.1
    case 2
        WarpIndex = 1;
    case 3
        WarpIndex = 2;
    case 4
        WarpIndex = [1,2]; % mv both robot No.1 and No.2
end

%% Get Rope Point Cloud at Test
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
% define region
valid_idx = (xyz_W(:,1) > 0) & (xyz_W(:,1) < 1.5) & (xyz_W(:,2) > -0.7) & (xyz_W(:,2) < 0.7) & (xyz_W(:,3) > 0) & (xyz_W(:,3) < 0.3);
xyz_W = xyz_W(valid_idx,:);

% only take the x-y plane points
PtCld_Test = xyz_W(:,1:2);

%% Warp Traj for Robot 1 or 2

disp('Please double check which robot motion needs to be warped!');
fig2_handle = figure(2); 
set(fig2_handle,'position', [962 562 958 434]);
rigidCompensate = 0;  %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[LTT_Data_Test, warp] = LTT_Warping_CPD(LTT_Data_Train, PtCld_Train, PtCld_Test, si , '2D', WarpIndex, rigidCompensate);

fig3_handle = figure(3);
set(fig3_handle,'position', [962 562 958 434]);
orig_fig = subplot(1,2,1); scatter(PtCld_Train(:,1), PtCld_Train(:,2),'r*'); title('Train');
warp_fig = subplot(1,2,2); scatter(PtCld_Test(:,1), PtCld_Test(:,2),'r*'); title('Test');
draw_grid([-0.5 0.8], [1 -0.7], warp, 20, orig_fig, warp_fig)
subplot(orig_fig); axis equal;xlim([-0.5,1]); ylim([-0.7,0.8]); drawnow;
subplot(warp_fig); axis equal; xlim([-0.5,1]); ylim([-0.7,0.8]); drawnow;

%% Remember to check collision if workpiece location is chaged substantially!
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

% Rope Manipulation Movement
Traj_Download_Run(LTT_Data_Test, si, 'Download', 'Run');

% Go back to home position
opt = {}; opt.is_dual_arm = true; opt.robot_idx = WarpIndex;
PosCmd = [[0,0,0,0,-90,0];[0,0,0,0,-90,0]]; 
tp_pos_run(si, PosCmd(WarpIndex,:), opt);

end

% Brake On
wasBrakeOff = brake_on_off(si.ParamSgnID, 'on');
wasStopped = tg_start_stop('stop');
