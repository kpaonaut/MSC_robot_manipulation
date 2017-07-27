
%% Training Phase

%% Place an object in the workspace


%% Get Object Point Cloud
% get point cloud from Kinect (change imThOpt.type color according to the object)
k2 = cam_open('Kinect');
imThOpt = []; imThOpt.plot_enb = 1; imThOpt.downsample = 1;
imThOpt.type = 'red';
imThOpt.ds_size = 500;
cableData = cam2pcl(imThOpt, k2);
cam_close('Kinect', k2);

% denoise
ptCld = pcdenoise(cableData.ptCld, 'NumNeighbors', 10, 'Threshold', 2 );

% Transform point cloud from camera coordinate to world coordinate
load('F:/TeTang/V4.0/sensing/T_W2K.mat');
xyz_K = ptCld.Location / 1000;   % change unit to m
xyz_W = (T_W2K(1:3,1:3)*xyz_K' + repmat(T_W2K(1:3,4),1,size(xyz_K,1)))';  
% define valid region
valid_idx = (xyz_W(:,1) > 0) & (xyz_W(:,1) < 1.5) & (xyz_W(:,2) > -0.7) & (xyz_W(:,2) < 0.7) & (xyz_W(:,3) > 0) & (xyz_W(:,3) < 0.3);
xyz_W = xyz_W(valid_idx,:);
    
% only take the x-y plane points
PtCld_Train = xyz_W(:,1:2);

% plot
scatter(PtCld_Train(:,1), PtCld_Train(:,2),'*');
xlabel('x');ylabel('y'); axis equal

%% Do LTT Teaching, log data by LTT_Data 
pause('Use slrtexplr to open teaching panel. Do LTT teaching');

%% log LTT_Data from Target PC
LTT_Data_Train = Load_LTT(si);
LTT_Data_Train = LTT_Data_Refine(LTT_Data_Train, si);

%% Save Point Cloud and LTT_Data_Train to /data/Rope/
save('F:/TeTang/TransferLearningbyCPD/data/Rope/PtCld_Train_RopeKnot_1.mat', PtCld_Train);
save('F:/TeTang/TransferLearningbyCPD/data/Rope/Traj_Train_RopeKnot_1.mat', LTT_Data_Train);




