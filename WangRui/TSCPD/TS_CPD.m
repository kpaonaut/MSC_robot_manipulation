%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%  TSM-RPM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Rui Wang, 08/03/2017       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Comunication with Ubuntu
% Run the following commands after setting up your PC's ros master IP, and connecting it to kinect
setenv('ROS_MASTER_URI','http://192.168.1.40:11311')
setenv('ROS_IP','192.168.1.90')
rosinit()

%% General Configuration
k = 2;
criticalSteps = 1; % --SUBJECT TO CHANGE; the total times of rope deformation
global LENGTH; LENGTH = 0.02; % -SUBJECT TO CHANGE; unit: m
WarpIndex = [1, 2]; % WarpIndex can be 1, 2, or [1, 2] --SUBJECT TO CHANGE

%% Load all training data and transform them into the world frame
load('F:\WANGRUI\V4.0\data\Rope\Traj_Train.mat'); % load the training trajectory

% load all training ropes:
for i = 1 : criticalSteps + 1
load(['F:\WANGRUI\V4.0\data\Rope\Rope_',num2str(i),'.mat']); % load the training data of trajectory
% transform points from kinect frame to world frame
points_W{i} = transformU2W(points_U, si); % training rope transformed
end

LTT_Data_Train.ReplayTime{1} = k * LTT_Data_Train.ReplayTime{1};
LTT_Data_Train.ReplayTime{2} = k * LTT_Data_Train.ReplayTime{2};

%% Generate trajectory
totalSteps = size(LTT_Data_Train.GrpCmd{1}, 1);

% Find the grasping point at training; also records rope and gripper state in each step
[graspPts, ManOrNot, picIdx, stepBegins] = FindClosestPts(LTT_Data_Train, WarpIndex, points_W);
% graspPts{idx}(i) is the index of rope node closest to grasping point during grasping step i for robot idx IF GraspOrNot
% stepBegins is the first small step of a critical step

%%
LTT_Data_Test = LTT_Data_Train; % temporarily init
stepBegins(criticalSteps + 1) = totalSteps + 1; 
for step = 1 : criticalSteps
    % Subsribe to ROS topic
    sub = rossubscriber('tracker/object'); % your PC, as ros master, should be publishing this topic (tracked obj) now
    received_data = receive(sub,3); % wait for at most 3 seconds.
    points = received_data.Rope.Nodes; % N*1 vector, each element: .X, .Y, .Z
    % transform test rope to world frame
    points_Test_U = [[points.X]', [points.Y]', [points.Z]'];
    points_Test_W = transformU2W(points_Test_U, si);
    points_Test_W = setOrder(points_Test_W);
%%
    points_Test_W = setOrder(points_Test_W); % set rope node enumerating order
    
    % CPD-Warp the robot trajectory in tangent space
    sb = stepBegins(step);
    se = stepBegins(step + 1) - 1;       
    rigidCompensate = 0;  % 0 or 1, whether we will use rigid transform (rotation) first
    points_train_q = getQ(points_W{step}(:, 1 : 2));
    points_test_q = getQ(points_Test_W(:, 1 : 2));
    train_goal_q = getQ(points_W{step+1}(:, 1 : 2));
    ts_train = [(1 : size(points_train_q, 1))', points_train_q]; % tangent space, convert q info into a 2D graph
    ts_test = [(1 : size(points_test_q, 1))', points_test_q]; % all x-axis is unscaled! [1, 2, 3...]
    train_goal_q = [(1 : size(train_goal_q, 1))', train_goal_q];
    
    sb = stepBegins(step);
    se = stepBegins(step + 1) - 1;    
    [LTT_Data_Test, warp] = CPD_warp(LTT_Data_Train, LTT_Data_Train, train_goal_q, points_Test_W, ts_train, ts_test, si , WarpIndex, rigidCompensate, graspPts, ManOrNot, sb, se, LENGTH);
    % Warping original rope to current rope finished!

    % visualize the warping of the original training rope and the test rope
    disp('Please double check which robot''s motion needs to be warped!');
%     fig2_handle = figure(2);
%     set(fig2_handle, 'position', [962 562 958 434]);
%     orig_fig = subplot(1,2,1); scatter(points_W(:, 1), points_W(:, 2), 'r*'); title('Train'); % plot the original rope 2-D shape
%     warp_fig = subplot(1,2,2); scatter(points_Test_W(:, 1), points_Test_W(:, 2), 'r*'); title('Test'); % plot the test rope
%     draw_grid([-0.5 0.8], [1 -0.7], warp, 20, orig_fig, warp_fig)
%     subplot(orig_fig); axis equal; xlim([-0.5,1]); ylim([-0.7,0.8]); drawnow;
%     subplot(warp_fig); axis equal; xlim([-0.5,1]); ylim([-0.7,0.8]); drawnow; % plote the grid
%     disp('Please double check which robot''s motion needs to be warped!');
    fig2_handle = figure(2);
    k = picIdx(sb);
    orig_fig = subplot(1,2,1); scatter(points_W{k}(:, 1), points_W{k}(:, 2), 'r*'); title('Train');xlim([-0.5,1]); ylim([-0.7,0.8]); % plot the original rope 2-D shape
    warp_fig = subplot(1,2,2); scatter(points_Test_W(:, 1), points_Test_W(:, 2), 'r*'); title('Test');xlim([-0.5,1]); ylim([-0.7,0.8]); % plot the test rope
%     draw_grid([-0.5 0.8], [1 -0.7], warp, 20, orig_fig, warp_fig)
%     subplot(orig_fig); axis equal; xlim([-0.5,1]); ylim([-0.7,0.8]); drawnow;
%     subplot(warp_fig); axis equal; xlim([-0.5,1]); ylim([-0.7,0.8]); drawnow; % plote the grid
    disp('Please double check which robot''s motion needs to be warped!');


    %% Run CFS, which displays an animation of fanuc robot following designed trajectory
    disp('======================================================================')
    % Remember to check collision if workpiece location is changed substantially!
    disp('======================================================================');
    fig1_handle = figure(2);
    set(fig1_handle, 'position', [962 42 958 434]);
    % save as LTT_Data_UCBtest for CFS check
    LTT_Data_UCBtest = LTT_Data_Test;
    save('F:\TeTang\V4.0\CFS_ver1.1\dataLTT\UCBTest.mat', 'LTT_Data_UCBtest');
    % CFS check
    CFS_Main
    input('Use CFS toolbox to check collision. Press any key to execute the motion!!!')


    %% Finally run the robot
    % Brake Off
    wasStopped = tg_start_stop('start');
    wasBrakeOff = brake_on_off(si.ParamSgnID, 'off');
    % Run!
    Traj_Download_Run(LTT_Data_Test, si, 'Download', 'Run');

end

%% Brake on
wasBrakeOff = brake_on_off(si.ParamSgnID, 'on');
wasStopped = tg_start_stop('stop');
