%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Warp LTT_Data by TSM-RPM function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Rui Wang, 08/02/2017       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [LTT_Data_Test_New] = TSM_RPM_Warp...
    (LTT_Data_Train, LTT_Data_Test, points_train_W, points_train_W_goal, points_Test_W, si, WarpIndex, graspPts, ManOrNot, stepBegin, stepEnd)
% MvOrNot == -1: move to a target without grasping; 1: move with grasping, i.e. manipulate; 0: do not move
% step: which step in LTT_Data_Test to create (to define the (x, y) position)
% opt == 0: space warp, not follow rope node; opt == 1: follow a point on rope

% Warping original rope to current rope in catesian space: (TPS-RPM) basic settings
frac = 1;
T_init = 0.1;
T_finalfac = 100;
disp_flag = 0; % do not display the whole iterating process
[c, d, m] = cMIX(points_train_W(:, 1:2), points_Test_W(:, 1:2), frac, T_init, T_finalfac, disp_flag); % !!! Unit MUST be meter, or TPS will not work
% For the expression above, c is w in the original paper; d is d in the paper; m is corr matrix
% Thus far, we have got corr matrix m, which is all we need from TPS-RPM
global LENGTH; % the length of each capsule in the rope, defined in TSM_RPM main function

% Apply the warping function to trajectory:
for idx = WarpIndex
%     train_traj = LTT_Data_Train.TCP_xyzwpr_W{idx}(step, 1:2); % coord of all key points on the traj
%     test_traj = ctps_warp_pts(train_traj, points_train_W(:, 1:2), c, d); % this function warps a series of points coord, (x, y)
    
%     LTT_Data_Test.TCP_xyzwpr_W{idx}(:, 1:2) = test_traj * 1000;
    % below: generates other values of LTT_Data_Test according to test_traj
    % FIXME: In this way, the direction of the gripper will never change,
    % which is wrong (but can work most of the time). Refer to TT's version of warping.
    for j = stepBegin : stepEnd % step range in one critical step
        if ManOrNot{idx}(j) == 0 % if a robot is neither moving to a point nor manipulating
            % stays still
            if j ~= 1
                LTT_Data_Test.TCP_xyzwpr_W{idx}(j, 1:2) = LTT_Data_Test.TCP_xyzwpr_W{idx}(j - 1, 1:2);
            end
        elseif ManOrNot{idx}(j) == -1 % if the robot aims at a static point on rope
            graspPtTrain = graspPts{idx}(j); % the index of grasping pt during training
            [~, graspPtTest] = max(m(graspPtTrain, :)); % the index of grasping pt in test | M(i, j) maximal
            LTT_Data_Test.TCP_xyzwpr_W{idx}(j, 1:2) = points_Test_W(graspPtTest, 1:2) * 1000; % ! in LTT, all data unit is mm
        else % if the robot is to manipulate the rope
            points_test_q = getOrientation(points_train_W_goal, m); % calculate degrees of test rope goal
            % ! Notice: when manipulating one end of the rope, should
            % integrate from the other end. Otherwise the robot won't move!
            num = graspPts{idx}(j);
            % FIXME! if one part of the rope is fixed by the other gripper,
            % should integrate from it. otherwise do the below:
            if num > size(points_Test_W, 1) / 2 % FIXME! grasp at near the end. integrate from the start
                grippingPointCoord = integrate...
                    (points_Test_W(1, 1:2), points_test_q, graspPts{idx}(j), LENGTH, 1); % calculate where the gripping point (x, y) on rope should be
                LTT_Data_Test.TCP_xyzwpr_W{idx}(j, 1:2) = grippingPointCoord * 1000; % unit must be mm!
            else % grasp at near the starting point, integrate from end. q also needs to change!
                grippingPointCoord = integrate...
                    (points_Test_W(end, 1:2), points_test_q - 180, graspPts{idx}(j), LENGTH, -1); % calculate where the gripping point (x, y) on rope should be
                LTT_Data_Test.TCP_xyzwpr_W{idx}(j, 1:2) = grippingPointCoord * 1000; % unit must be mm!
            end
        end
        LTT_Data_Test.TCP_T_W{idx}(:, :, j) = xyzwpr2T(LTT_Data_Test.TCP_xyzwpr_W{idx}(j, :));
        LTT_Data_Test.TCP_T_B{idx}(:, :, j) = FrameTransform(LTT_Data_Test.TCP_T_W{idx}(:, :, j), 'T', 'W2B', si, idx);
        LTT_Data_Test.TCP_xyzwpr_B{idx}(j, :) = T2xyzwpr(LTT_Data_Test.TCP_T_B{idx}(:, :, j));
        LTT_Data_Test.DesJntPos{idx}(j, :) = fanucikine(LTT_Data_Test.TCP_xyzwpr_B{idx}(j, :), si.ri{idx}, LTT_Data_Train.DesJntPos{idx}(j, :));
    end
end
LTT_Data_Test_New = LTT_Data_Test;
% warp = @(x) ctps_warp_pts(x, points_train_W(:, 1:2), c, d); % the function handle of cpd-warping
end