%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Warp LTT_Data by TPS-RPM function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Rui Wang, 08/01/2017       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [LTT_Data_Test, warp] = TPS_RPM_Warp(LTT_Data_Train, points_train_W, points_Test_W, si, WarpIndex);

% Warping original rope to current rope:
frac = 1;
T_init = 0.1;
T_finalfac = 100;
disp_flag = 0; % do not display the whole iterating process
[c, d] = cMIX(points_train_W(:, 1:2), points_Test_W(:, 1:2), frac, T_init, T_finalfac, disp_flag); 
% For the expression above, c is w in the original paper; d is d in the paper; m is corr matrix
% Apply the warping function to trajectory:
LTT_Data_Test = LTT_Data_Train; % temporary init
for idx = WarpIndex
    train_traj = LTT_Data_Train.TCP_xyzwpr_W{idx}(:, 1:2)./1000; % coord of all key points on the traj
    test_traj = ctps_warp_pts(train_traj, points_train_W(:, 1:2), c, d); % this function warps a series of points coord, (x, y)

    LTT_Data_Test.TCP_xyzwpr_W{idx}(:, 1:2) = test_traj * 1000;
    % below: generates other values of LTT_Data_Test according to test_traj
    % FIXME: In this way, the direction of the gripper will never change,
    % which is wrong (but can work most of the time). Refer to TT's version of warping.
    steps = size(train_traj, 1);
    for j = 1 : steps
        LTT_Data_Test.TCP_T_W{idx}(:, :, j) = xyzwpr2T(LTT_Data_Test.TCP_xyzwpr_W{idx}(j, :));
        LTT_Data_Test.TCP_T_B{idx}(:, :, j) = FrameTransform(LTT_Data_Test.TCP_T_W{idx}(:, :, j), 'T', 'W2B', si, idx);
        LTT_Data_Test.TCP_xyzwpr_B{idx}(j, :) = T2xyzwpr(LTT_Data_Test.TCP_T_B{idx}(:, :, j));
        LTT_Data_Test.DesJntPos{idx}(j, :) = fanucikine(LTT_Data_Test.TCP_xyzwpr_B{idx}(j, :), si.ri{idx}, LTT_Data_Train.DesJntPos{idx}(j, :));
    end
end

warp = @(x) ctps_warp_pts(x, points_train_W(:, 1:2), c, d); % the function handle of cpd-warping
