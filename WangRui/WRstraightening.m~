%%%    Berkeley MSC Lab  %%%
%    Created by Rui Wang   %
%         7/27/2017        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% comunication with Ubuntu
%Run the following commands after setting up your PC's ros master IP, and connecting it to kinect
setenv('ROS_MASTER_URI','http://192.168.1.40:11311')
setenv('ROS_IP','192.168.1.90')
rosinit()

%% Brake Off
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(si.ParamSgnID, 'off');

%% Subsribe to ROS topic
sub = rossubscriber('tracker/object'); % your PC, as ros master, should be publishing this topic (tracked obj) now
received_data = receive(sub,3); % wait for at most 3 seconds.
points = received_data.Rope.Nodes; % N*1 vector, each element: .X, .Y, .Z
N = size(points,1); % total num of nodes on a rope

%% Load all points and transform them into the world frame
clear points_Test_U
T_W2U = transl((22.5 * 2.54 + 2.486 * 5) / 100, -2.486 * 2.5 / 100, 0) * trotz(-pi / 2); % transformation matrix, World to Kinect
T_B2U = inv(si.ri{1}.T_B2W) * T_W2U; % transformation matrix, Robot Base to Kinect
load('F:\WANGRUI\V4.0\data\Rope\Traj_Train_Straightening.mat'); % load the training data
load('F:\WANGRUI\V4.0\data\Rope\Bullet_Train_Straightening0.mat');

% transform points from kinect frame to world frame
% N1= size(points_U, 1)
% N2=size(points_Test_U, 1)
N1= size(points_U, 1);
%%%
if N1 < N
    N = N1;
else
    N1 = N;
end
points = points(1 : N, :);
points_U = points_U(1 : N, :);
%%%
for i=1:N
  points_Test_U(i, 1)=points(i).X;
  points_Test_U(i, 2)=points(i).Y;
  points_Test_U(i, 3)=points(i).Z;
end
points_U = [points_U, ones(N1, 1)];
points_W = (T_W2U * points_U')';
points_W = points_W(:, 1 : 3);
points_U = points_U(:, 1 : 3); % all training points transformed to world finished
points_Test_U = [points_Test_U, ones(N, 1)];
points_Test_W = (T_W2U * points_Test_U')';
points_Test_W = points_Test_W(:, 1 : 3);
points_Test_U = points_Test_U(:, 1 : 3); % testing points transformed to world finished

k = 2;
WarpIndex = [1, 2];
LTT_Data_Train.ReplayTime{1} = k * LTT_Data_Train.ReplayTime{1};
LTT_Data_Train.ReplayTime{2} = k * LTT_Data_Train.ReplayTime{2};

%% CPD-Warp the robot trajectory
rigidCompensate = 0;  % ???
[Z_Data_Test2, warp] = LTT_Warping_CPD(LTT_Data_Train, points_W(:, 1 : 2), points_Test_W(:, 1 : 2), si , '2D', WarpIndex, rigidCompensate);
% Warping original rope to current rope finished!

disp('Please double check which robot''s motion needs to be warped!');
fig2_handle = figure(2);
set(fig2_handle, 'position', [962 562 958 434]);

% visualize the warping of the original training rope and the test rope
fig3_handle = figure(3);
set(fig3_handle, 'position', [962 562 958 434]);
orig_fig = subplot(1,2,1); scatter(points_W(:, 1), points_W(:, 2), 'r*'); title('Train'); % plot the original rope 2-D shape
warp_fig = subplot(1,2,2); scatter(points_Test_W(:, 1), points_Test_W(:, 2), 'r*'); title('Test'); % plot the test rope
draw_grid([-0.5 0.8], [1 -0.7], warp, 20, orig_fig, warp_fig)
subplot(orig_fig); axis equal; xlim([-0.5,1]); ylim([-0.7,0.8]); drawnow;
subplot(warp_fig); axis equal; xlim([-0.5,1]); ylim([-0.7,0.8]); drawnow; % plote the grid

% Run CFS, which displays an animation of fanuc robot following designed trajectory
disp('======================================================================')
% Remember to check collision if workpiece location is changed substantially!
disp('======================================================================');
fig1_handle = figure(2);
set(fig1_handle, 'position', [962 42 958 434]);
% save as LTT_Data_UCBtest for CFS check
LTT_Data_UCBtest = Z_Data_Test2;
save('F:\WANGRUI\V4.0\CFS_ver1.1\dataLTT\UCBTest.mat', 'LTT_Data_UCBtest');
% CFS check
CFS_Main
input('Use CFS toolbox to check collision. Press any key to execute the motion!!!')

%% Finally run the robot
% rigidCompensate = 0;
% [Z_Data_Test2, warp] = LTT_Warping_CPD(LTT_Data_Train, points_W(:,1:2), points_Test_W(:,1:2), si , '2D', WarpIndex, rigidCompensate);

Traj_Download_Run(Z_Data_Test2, si, 'Download', 'Run');
