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
% Set the options
%    %(default 3) Regularization weight.
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

% if rigidCompensate == true
%     warp = @(x) warp(warp_rigid(x));
% end
Z = warp(goal')';

%% DEBUG
 a = warp([400; 87.9])';
%% create New_LTT_Data after warping
num = 0;
for idx = robot_idx
    for j = stepBegin : stepEnd % step range in one critical step
        if ManOrNot{idx}(j) == 0 % if a robot is neither moving to a point nor manipulating
            % stays still
            if j ~= 1
                LTT_Data_Test.TCP_xyzwpr_W{idx}(j, 1:3) = LTT_Data_Test.TCP_xyzwpr_W{idx}(j - 1, 1:3);
            end % if j == 1, the robot stays still, at origin!
            LTT_Data_Test.TCP_T_W{idx}(:, :, j) = xyzwpr2T(LTT_Data_Test.TCP_xyzwpr_W{idx}(j, :));
            LTT_Data_Test.TCP_T_B{idx}(:, :, j) = FrameTransform(LTT_Data_Test.TCP_T_W{idx}(:, :, j), 'T', 'W2B', si, idx);
            LTT_Data_Test.TCP_xyzwpr_B{idx}(j, :) = T2xyzwpr(LTT_Data_Test.TCP_T_B{idx}(:, :, j));
            LTT_Data_Test.DesJntPos{idx}(j, :) = fanucikine(LTT_Data_Test.TCP_xyzwpr_B{idx}(j, :), si.ri{idx}, LTT_Data_Train.DesJntPos{idx}(j, :));
        else
            if ManOrNot{idx}(j)==1, manStep = [idx, j]; end
            LTT_Data_Test.TCP_xyzwpr_W{idx}(j, 1:2) = [warp([LTT_Data_Train.TCP_xyzwpr_W{idx}(j, 1:2)]'/1000)*1000]'; % unit must be mm!    
            %LTT_Data_Test = warpFn(LTT_Data_Train, idx, j, warp, si);
            LTT_Data_Test.TCP_T_W{idx}(:, :, j) = xyzwpr2T(LTT_Data_Test.TCP_xyzwpr_W{idx}(j, :));
            LTT_Data_Test.TCP_T_B{idx}(:, :, j) = FrameTransform(LTT_Data_Test.TCP_T_W{idx}(:, :, j), 'T', 'W2B', si, idx);
            LTT_Data_Test.TCP_xyzwpr_B{idx}(j, :) = T2xyzwpr(LTT_Data_Test.TCP_T_B{idx}(:, :, j));
            LTT_Data_Test.DesJntPos{idx}(j, :) = fanucikine(LTT_Data_Test.TCP_xyzwpr_B{idx}(j, :), si.ri{idx}, LTT_Data_Train.DesJntPos{idx}(j, :));
        end
    end
end

%% add height after grasping
idx = manStep(1);
j = manStep(2);
TCP_xyzwpr_W{idx} = LTT_Data_Test.TCP_xyzwpr_W{idx}(j-1, :);
TCP_xyzwpr_W{idx}(1, 3) = TCP_xyzwpr_W{idx}(1, 3)+50;
TCP_T_W{idx} = xyzwpr2T(TCP_xyzwpr_W{idx});
TCP_T_B{idx} = FrameTransform(TCP_T_W{idx}, 'T', 'W2B', si, idx);
TCP_xyzwpr_B{idx} = T2xyzwpr(TCP_T_B{idx});
DesJntPos{idx} = fanucikine(TCP_xyzwpr_B{idx}, si.ri{idx}, LTT_Data_Test.DesJntPos{idx}(j-1, :));

TCP_xyzwpr_W{3-idx} = LTT_Data_Test.TCP_xyzwpr_W{3-idx}(j-1, :);% not lift up
TCP_T_W{3-idx} = xyzwpr2T(TCP_xyzwpr_W{3-idx});
TCP_T_B{3-idx} = FrameTransform(TCP_T_W{3-idx}, 'T', 'W2B', si, 3-idx);
TCP_xyzwpr_B{3-idx} = T2xyzwpr(TCP_T_B{3-idx});
DesJntPos{3-idx} = fanucikine(TCP_xyzwpr_B{3-idx}, si.ri{3-idx}, LTT_Data_Test.DesJntPos{3-idx}(j-1, :));

for idx = 1:2
    LTT_Data_Test.TCP_xyzwpr_W{idx}(1 : j-stepBegin, :) = LTT_Data_Test.TCP_xyzwpr_W{idx}(stepBegin:j-1, :);
    LTT_Data_Test.TCP_xyzwpr_W{idx}(j-stepBegin+2 : stepEnd-stepBegin+2, :) = LTT_Data_Test.TCP_xyzwpr_W{idx}(j:stepEnd, :);
    LTT_Data_Test.TCP_xyzwpr_W{idx}(j-stepBegin+1, :) = TCP_xyzwpr_W{idx};

    LTT_Data_Test.DesJntPos{idx}(1 : j-stepBegin, :) = LTT_Data_Test.DesJntPos{idx}(stepBegin:j-1, :);
    LTT_Data_Test.DesJntPos{idx}(j-stepBegin+2 : stepEnd-stepBegin+2, :) = LTT_Data_Test.DesJntPos{idx}(j:stepEnd, :);
    LTT_Data_Test.DesJntPos{idx}(j-stepBegin+1, :) = DesJntPos{idx};
    
    LTT_Data_Test.ReplayTime{idx} = LTT_Data_Test.ReplayTime{idx}(stepBegin:stepEnd-1, :);
    LTT_Data_Test.ReplayTime{idx} = [LTT_Data_Test.ReplayTime{idx}; LTT_Data_Test.ReplayTime{idx}(1)];% append one replay time
    
    LTT_Data_Test.GrpCmd{idx}(1 : j-stepBegin, :) = LTT_Data_Test.GrpCmd{idx}(stepBegin:j-1, :);
    LTT_Data_Test.GrpCmd{idx}(j-stepBegin+2 : stepEnd-stepBegin+2, :) = LTT_Data_Test.GrpCmd{idx}(j:stepEnd, :);
    LTT_Data_Test.GrpCmd{idx}(j-stepBegin+1, :) = [0 0 0 0 0 1]; % grip keep!
end