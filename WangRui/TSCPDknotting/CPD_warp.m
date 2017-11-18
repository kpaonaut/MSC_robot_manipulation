%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Warp LTT_Data by CPD function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 04/17/2017       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [LTT_Data_Test, warp] = CPD_warp(LTT_Data_Train, LTT_Data_Test_old, train_goal_q, points_Test_W, train_q, test_q, si, ...
    robot_idx, rigidCompensate, graspPts, ManOrNot, stepBegin, stepEnd, LENGTH, gotoinit)

% actually train_q and test_q are degrees here! in tangent space!
LTT_Data_Test = LTT_Data_Test_old; % init

%% specify which arm to warp
if nargin < 6
    robot_idx = [1,2];
end

%% Deal with the issue of angles jumping from -179 to +180.
% FIXME!!! might need rigid compensate for a 360-degree-translation!
test_q = continuous(test_q);
train_q = continuous(train_q); % make the degree seires continuous
train_goal_q = continuous(train_goal_q);
X = test_q;
Y = train_q;

%% CPD
%rigidCompensate = 1;
% rigid rotation first
if rigidCompensate == true
    opt = [];
    opt.method = 'rigid'; % use rigid registration
    opt.fgt = 0;
    opt.viz = 1;          % show every iteration
    opt.rot = 1;
    opt.normalize = 0;
    opt.scale = 0;
    %registering Y to X
    Transform_rigid = cpd_register(X,Y,opt);
    %generate warp function handle
    warp_rigid = @(x) (cpd_transform(x', Transform_rigid))';
    Y = Transform_rigid.Y;
end

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
Transform = cpd_register([X(:, 1)*10, X(:, 2)], [Y(:, 1)*10, Y(:, 2)], opt);% registering Y to X
% generate warp function handle
warp = @(x) (cpd_transform(x', Transform))'; % x is column

if rigidCompensate == true
    warp = @(x) warp(warp_rigid([x(:, 1)*10, x(:, 2)]));
end
Z = warp([[Y(:, 1)*10, Y(:, 2)]]')';
P = CPD_PMatrix([X(:, 1)*10, X(:, 2)], Z, Transform.sigma2, opt.outliers, 2); % Pm,n: the P that Xn is an observation of Ym, not other Y

checkP = sum(P, 1);
%% DEBUG
%a = warp([400; 87.9])';
%% create New_LTT_Data after warping
num = 0;
aimStep = [];
for idx = robot_idx
    for j = stepBegin : stepEnd % step range in one critical step
        if ManOrNot{idx}(j) == 0 % if a robot is neither moving to a point nor manipulating
            % stays still
            if j ~= 1
                LTT_Data_Test.TCP_xyzwpr_W{idx}(j, 1:2) = LTT_Data_Test.TCP_xyzwpr_W{idx}(j - 1, 1:2);
            end % if j == 1, the robot stays still, at origin!
        elseif ManOrNot{idx}(j) == -1 % if the robot aims at a static point on rope
             graspPtTrain = graspPts{idx}(j); % the index of grasping pt during training
%             graspPtTest = warp([train_q(graspPtTrain, 1)*10, train_q(graspPtTrain, 2)]'); % the TS coord of cpd-warped grasping pt
%             graspPtTest = graspPtTest';
%             num = dsearchn(test_q(:, 1:2), graspPtTest(1, :)); % the index of the closest point in TS
            [~ ,num] = max(P(graspPtTrain, :));
            LTT_Data_Test.TCP_xyzwpr_W{idx}(j, 1:2) = points_Test_W(num, 1:2) * 1000; % unit must be mm!
            LTT_Data_Test.TCP_xyzwpr_W{idx}(j, 3) = 65;
            aimStep = [aimStep; idx j];
        else % if the robot is to manipulate the rope
            % num = graspPts{idx}(j); % index of grasping points here!
            diff = train_q(1, 2) - train_goal_q(1, 2);
            if diff > 300
                train_goal_q(:, 2) = train_goal_q(:, 2) + 360;
            elseif diff < -300
                train_goal_q(:, 2) = train_goal_q(:, 2) - 360;
            end
            %[~ ,num] = max(P(graspPts{idx}(j), :));
            % num should be the same as the previous one when the gripper aims at sth!
            % test_goal_q = warp(train_goal_q')'; % get discrete dots!
            % using warping
            test_goal_q = P'*(train_goal_q(:, 2));
            %test_goal_q = resize(test_goal_q, size(test_q, 1)); % interpolate values to get a new set of points in TS, with x == 1..nTest
            % should integrate from it. otherwise do the below:
            if num > size(test_q, 1) / 2 % FIXME! grasp at near the end. integrate from the start
                grippingPointCoord = integrate...
                    (points_Test_W(1, 1:2), test_goal_q, num, LENGTH, 1); % calculate where the gripping point (x, y) on rope should be
            else % grasp at near the starting point, integrate from end. q also needs to change!
                grippingPointCoord = integrate...
                    (points_Test_W(end, 1:2), test_goal_q - 180, num, LENGTH, -1); % calculate where the gripping point (x, y) on rope should be
            end
            grippingPointCoord = grippingPointCoord + [-0.00, 0]; % HARD CODE: deviation!(offset)
            if gotoinit{idx}(j) == 0
                LTT_Data_Test.TCP_xyzwpr_W{idx}(j, 1:2) = grippingPointCoord * 1000; % unit must be mm!  
            end % otherwise same as train
            recoverPlot([0, 0], train_q(:, 2), LENGTH, 1); % just to check shape, position doesn't matter!
            recoverPlot([0, 0], test_goal_q, LENGTH, 1);
            manStep = [idx, j]; % record the step of manipulation
        end
        if gotoinit{idx}(j) == 0
            LTT_Data_Test.TCP_T_W{idx}(:, :, j) = xyzwpr2T(LTT_Data_Test.TCP_xyzwpr_W{idx}(j, :));
            LTT_Data_Test.TCP_T_B{idx}(:, :, j) = FrameTransform(LTT_Data_Test.TCP_T_W{idx}(:, :, j), 'T', 'W2B', si, idx);
            LTT_Data_Test.TCP_xyzwpr_B{idx}(j, :) = T2xyzwpr(LTT_Data_Test.TCP_T_B{idx}(:, :, j));
            if j-1 ~= 0
                LTT_Data_Test.DesJntPos{idx}(j, :) = fanucikine(LTT_Data_Test.TCP_xyzwpr_B{idx}(j, :), si.ri{idx}, LTT_Data_Test.DesJntPos{idx}(j-1, :));
            else
                LTT_Data_Test.DesJntPos{idx}(j, :) = fanucikine(LTT_Data_Test.TCP_xyzwpr_B{idx}(j, :), si.ri{idx}, LTT_Data_Train.DesJntPos{idx}(j, :));
            end
        end % otherwise keep the same as train
    end

end
%% manipulation step: uplift 13 cm!
idx = manStep(1);
j = manStep(2);
TCP_xyzwpr_W{idx} = LTT_Data_Test.TCP_xyzwpr_W{idx}(j-1, :);
if stepBegin ~=1
TCP_xyzwpr_W{idx}(1, 3) = TCP_xyzwpr_W{idx}(1, 3)+130;% lift up; 1st step, do not lift up
end
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
    LTT_Data_Test_tmp.TCP_xyzwpr_W{idx}(1 : j-stepBegin, :) = LTT_Data_Test.TCP_xyzwpr_W{idx}(stepBegin:j-1, :);
    LTT_Data_Test_tmp.TCP_xyzwpr_W{idx}(j-stepBegin+2 : stepEnd-stepBegin+2, :) = LTT_Data_Test.TCP_xyzwpr_W{idx}(j:stepEnd, :);
    LTT_Data_Test_tmp.TCP_xyzwpr_W{idx}(j-stepBegin+1, :) = TCP_xyzwpr_W{idx};

    LTT_Data_Test_tmp.DesJntPos{idx}(1 : j-stepBegin, :) = LTT_Data_Test.DesJntPos{idx}(stepBegin:j-1, :);
    LTT_Data_Test_tmp.DesJntPos{idx}(j-stepBegin+2 : stepEnd-stepBegin+2, :) = LTT_Data_Test.DesJntPos{idx}(j:stepEnd, :);
    LTT_Data_Test_tmp.DesJntPos{idx}(j-stepBegin+1, :) = DesJntPos{idx};
    
    LTT_Data_Test_tmp.ReplayTime{idx} = LTT_Data_Test.ReplayTime{idx}(stepBegin:stepEnd-1, :);
    LTT_Data_Test_tmp.ReplayTime{idx} = [LTT_Data_Test_tmp.ReplayTime{idx}; LTT_Data_Test_tmp.ReplayTime{idx}(1)];% append one replay time
    
    LTT_Data_Test_tmp.GrpCmd{idx}(1 : j-stepBegin, :) = LTT_Data_Test.GrpCmd{idx}(stepBegin:j-1, :);
    LTT_Data_Test_tmp.GrpCmd{idx}(j-stepBegin+2 : stepEnd-stepBegin+2, :) = LTT_Data_Test.GrpCmd{idx}(j:stepEnd, :);
    LTT_Data_Test_tmp.GrpCmd{idx}(j-stepBegin+1, :) = [0 0 0 0 0 1]; % grip keep!
end

for idx=1:2
for i = 1:size(LTT_Data_Test_tmp.DesJntPos{idx}, 1) % angle adjust
    if LTT_Data_Test_tmp.DesJntPos{idx}(i, 6)>360
        LTT_Data_Test_tmp.DesJntPos{idx}(i, 6) = LTT_Data_Test_tmp.DesJntPos{idx}(i, 6) - 360;
    elseif LTT_Data_Test_tmp.DesJntPos{idx}(i, 6)<-360
        LTT_Data_Test_tmp.DesJntPos{idx}(i, 6) = LTT_Data_Test_tmp.DesJntPos{idx}(i, 6) + 360;
    end
end
end
LTT_Data_Test_tmp = gotoLift(LTT_Data_Test_tmp, si); % add one step at the first step of each critical step!
LTT_Data_Test_tmp = angleInterp(LTT_Data_Test_tmp); % deal with possible angles > 360
LTT_Data_Test = LTT_Data_Test_tmp;
