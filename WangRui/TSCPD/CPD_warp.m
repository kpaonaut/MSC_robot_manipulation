%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Warp LTT_Data by CPD function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 04/17/2017       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [LTT_Data_Test, warp] = CPD_warp(LTT_Data_Train, LTT_Data_Test_old, train_goal_q, points_Test_W, train_q, test_q, si, ...
    robot_idx, rigidCompensate, graspPts, ManOrNot, stepBegin, stepEnd, LENGTH)

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
Z = warp(Y')';
P = CPD_PMatrix([X(:, 1)*10, X(:, 2)], Z, Transform.sigma2, opt.outliers, 2); % Pm,n: the P that Xn is an observation of Ym, not other Y

checkP = sum(P, 1);
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
        elseif ManOrNot{idx}(j) == -1 % if the robot aims at a static point on rope
             graspPtTrain = graspPts{idx}(j); % the index of grasping pt during training
%             graspPtTest = warp([train_q(graspPtTrain, 1)*10, train_q(graspPtTrain, 2)]'); % the TS coord of cpd-warped grasping pt
%             graspPtTest = graspPtTest';
%             num = dsearchn(test_q(:, 1:2), graspPtTest(1, :)); % the index of the closest point in TS
            [~ ,num] = max(P(:, graspPtTrain));
            LTT_Data_Test.TCP_xyzwpr_W{idx}(j, 1:2) = points_Test_W(num, 1:2) * 1000; % unit must be mm!
        else % if the robot is to manipulate the rope
            % num = graspPts{idx}(j); % index of grasping points here!
            diff = train_q(1, 2) - train_goal_q(1, 2);
            if diff > 300
                train_goal_q(:, 2) = train_goal_q(:, 2) + 360;
            elseif diff < -300
                train_goal_q(:, 2) = train_goal_q(:, 2) - 360;
            end
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
            LTT_Data_Test.TCP_xyzwpr_W{idx}(j, 1:2) = grippingPointCoord * 1000; % unit must be mm!    
            
            recoverPlot([0, 0], train_q(:, 2), LENGTH, 1); % just to check shape, position doesn't matter!
            recoverPlot([0, 0], test_goal_q, LENGTH, 1);
        end
        LTT_Data_Test.TCP_T_W{idx}(:, :, j) = xyzwpr2T(LTT_Data_Test.TCP_xyzwpr_W{idx}(j, :));
        LTT_Data_Test.TCP_T_B{idx}(:, :, j) = FrameTransform(LTT_Data_Test.TCP_T_W{idx}(:, :, j), 'T', 'W2B', si, idx);
        %LTT_Data_Test.DesJntPos{idx}(j, :) = fanucikine(LTT_Data_Test.TCP_xyzwpr_B{idx}(j, :), si.ri{idx}, LTT_Data_Train.DesJntPos{idx}(j, :));
        % !!FIXME the above sentence is commented due to mac cannot running
        % the strange cmix function
        % must uncomment on windows!!!
    end
end