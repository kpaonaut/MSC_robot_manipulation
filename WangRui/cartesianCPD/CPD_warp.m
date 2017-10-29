%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Warp LTT_Data by CPD function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 04/17/2017       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [LTT_Data_Test, warp] = CPD_warp(LTT_Data_Train, LTT_Data_Test_old, goal, train, test, si, ...
    robot_idx, rigidCompensate, ManOrNot, stepBegin, stepEnd)

% actually train_q and test_q are degrees here! in tangent space!
LTT_Data_Test = LTT_Data_Test_old; % init

X = test;
Y = train;

%% CPD

% non rigid 
% Set the options
%    %(default 3) Regularization weight.
% opt.method='nonrigid';
% opt.viz=1;              % show every iteration
% opt.outliers=0.5;         % don't account for outliers
% opt.normalize=1;        % normalize to unit variance and zero mean before registering (default)
% opt.corresp=1;          % compute correspondence vector at the end of registration (not being estimated by default)
% opt.max_it=100;         % max number of iterations
% opt.tol=1e-9;          % tolerance!
% registering Y to X
opt = [];
opt.method = 'nonrigid'; % whether or not to use rigid registration
opt.fgt = 0;
opt.viz = 1;          % show every iteration
opt.beta = 1;      %(default 2) Gaussian smoothing filter size. Forces rigidity.
opt.lambda = 10;
% original: beta=1; lambda = 5;
opt.max_it=100;         % max number of iterations
opt.tol=1e-9;
opt.outliers = 0.000;
Transform = cpd_register(X, Y, opt);% registering Y to X
% generate warp function handle
warp = @(x) (cpd_transform(x', Transform))'; % x is column

if rigidCompensate == true
    warp = @(x) warp(warp_rigid([x(:, 1)*10, x(:, 2)]));
end
Z = warp(goal')';

%% DEBUG
%a = warp([400; 87.9])';
%% create New_LTT_Data after warping
num = 0;
for idx = robot_idx
    for j = stepBegin : stepEnd % step range in one critical step
        if ManOrNot{idx}(j) == 0 % if a robot is neither moving to a point nor manipulating
            % stays still
            if j ~= 1
                LTT_Data_Test.TCP_xyzwpr_W{idx}(j, 1:2) = LTT_Data_Test.TCP_xyzwpr_W{idx}(j - 1, 1:2);
            end % if j == 1, the robot stays still, at origin!
%         elseif ManOrNot{idx}(j) == -1 % if the robot aims at a static point on rope
%              graspPtTrain = graspPts{idx}(j); % the index of grasping pt during training
% %             graspPtTest = warp([train_q(graspPtTrain, 1)*10, train_q(graspPtTrain, 2)]'); % the TS coord of cpd-warped grasping pt
% %             graspPtTest = graspPtTest';
% %             num = dsearchn(test_q(:, 1:2), graspPtTest(1, :)); % the index of the closest point in TS
%             [~ ,num] = max(P(graspPtTrain, :));
%             LTT_Data_Test.TCP_xyzwpr_W{idx}(j, 1:2) = points_Test_W(num, 1:2) * 1000; % unit must be mm!
%         else % if the robot is to manipulate the rope
%             % using warping
%             test_goal_q = P'*(train_goal_q(:, 2));
        else
            LTT_Data_Test.TCP_xyzwpr_W{idx}(j, 1:2) = [warp([LTT_Data_Train.TCP_xyzwpr_W{idx}(j, 1:2)]'/1000)*1000]'; % unit must be mm!    
%             recoverPlot([0, 0], train_q(:, 2), LENGTH, 1); % just to check shape, position doesn't matter!
%             recoverPlot([0, 0], test_goal_q, LENGTH, 1);
        end
        LTT_Data_Test.TCP_T_W{idx}(:, :, j) = xyzwpr2T(LTT_Data_Test.TCP_xyzwpr_W{idx}(j, :));
        LTT_Data_Test.TCP_T_B{idx}(:, :, j) = FrameTransform(LTT_Data_Test.TCP_T_W{idx}(:, :, j), 'T', 'W2B', si, idx);
        LTT_Data_Test.TCP_xyzwpr_B{idx}(j, :) = T2xyzwpr(LTT_Data_Test.TCP_T_B{idx}(:, :, j));
        LTT_Data_Test.DesJntPos{idx}(j, :) = fanucikine(LTT_Data_Test.TCP_xyzwpr_B{idx}(j, :), si.ri{idx}, LTT_Data_Train.DesJntPos{idx}(j, :));
    end
end