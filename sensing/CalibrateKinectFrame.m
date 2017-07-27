% Te Tang, UC Berkeley, 2017/04/16
% Take Kinect v2 capture images and calibrate the transformation matrix T_World2Kinect


%% Take N flipped rgb pictures to Calibrate Camera, save to opt.datapath folder
% For the first picture, please place the chessboard at the pose where you want to define the world coorindate
% For the following N-1 pictures, randomly change the pose of the chessboard for camera calibration
k2 = cam_open('Kinect');
N = 20;
opt = []; opt.types = 'c'; opt.plot_enb = 1; opt.savefig = 1; opt.datapath = 'F:/TeTang/V4.0/data/fig/';
opt.CalibrationFlip = 1; opt.colorScale = 1;
for i = 1:N
    camData = GetKinectData(k2, opt);
    pause(2);
    disp(['Picture No.: ', num2str(i)]);
end

cam_close('Kinect', k2);

%% Open Matlab APP:  Camera Calibrator, to do calibration
pause('Please use Matlab APP, Camera Calibrator, to do calibration')
% (1) Import the captured figure
% (2) Input 24.86mm for chessboard size (measure the real size. The value might change using differing printer configuration)
% (3) Output the variable "cameraParams" to workspace
% (4) If the cameraParams.FocalLength (in pixels) deviates too much with the value in the depthToCould.m, then change depthToCloud.m variable.

%% Calibrate Transforamtion matrix
pause('Run following codes only if the previous step is finished.')
index = 1;
% The t, R definition is quite different from robotics 
% refer to: https://www.mathworks.com/help/vision/ref/extrinsics.html#outputarg_translationVector
t = cameraParams.TranslationVectors(index,:)'./1000;   % unit: m 
R = cameraParams.RotationMatrices(:,:,index);
T_Board2Camera = [[R,-R*t]; [0,0,0,1]];

T_Camera2Kinect = [-1  0  0  0;...
          0 -1  0  0;...
          0  0  1  0;...
          0  0  0  1];
T_World2Board = [ 1  0   0  31/1000;...
          0 -1   0  0;...
          0  0  -1  0;...
          0  0   0  1];
T_W2K = T_World2Board*T_Board2Camera*T_Camera2Kinect;      % unit: m

%% save T_W2K for futher usage
flag = input('whether save this T_W2K to mat file for futher usage?');
if flag
    save('F:/TeTang/V4.0/sensing/T_W2K.mat', 'T_W2K');
end

%% Test (Get Point Cloud of Red Object and check whether T_W2K performs well)
k2 = cam_open('Kinect');
imThOpt = []; imThOpt.plot_enb = 1;
cableData = cam2pcl(imThOpt, k2);
cam_close('Kinect', k2);
xyz_K = cableData.ptCld.Location / 1000;   % change unit to m
xyz_W = (T_W2K(1:3,1:3)*xyz_K' + repmat(T_W2K(1:3,4),1,size(xyz_K,1)))';  % change unit to m
pcshow(xyz_W)
xlabel('x');ylabel('y');zlabel('z');
