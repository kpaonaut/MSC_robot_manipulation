%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Main File for Wire Harness Task 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% get Aruco Markers' pose. Return error if the desired marker is not found
ArucoMarker = getArucoMarker(si, true, true);
drawnow;
WhiteNotch_02_T_W = ArucoMarker.T_w_adjust{ArucoMarker.ids == 2};
WhiteNotch_04_T_W = ArucoMarker.T_w_adjust{ArucoMarker.ids == 4};
BlackSlot_01_T_W = ArucoMarker.T_w_adjust{ArucoMarker.ids == 1};
BlackSlot_03_T_W = ArucoMarker.T_w_adjust{ArucoMarker.ids == 3};
BlackSlot_05_T_W = ArucoMarker.T_w_adjust{ArucoMarker.ids == 5};
TapingTool_T_W = ArucoMarker.T_w{ArucoMarker.ids == 0};

%% check detection result
disp('======================================================================')
input('Check the ArucoMarker detection picture. Press any key to continue...')

%% Load motion primitives. Warp trajectory according to new feature points.
LTT_Data_WhiteNotch = Load_LTT_WhiteNotch(si);
LTT_Data_BlackSlot = Load_LTT_BlackSlot(si);
LTT_Data_BlackSlot_End = Load_LTT_BlackSlot_End(si);
LTT_Data_ApproachTaping = Load_LTT_ApproachTaping(si);

% get each objects' feature points from marker pose
WhiteNotch_02_FeaturePoint = getFeaturePointfromMarker(WhiteNotch_02_T_W, 'WhiteNotch');
WhiteNotch_04_FeaturePoint = getFeaturePointfromMarker(WhiteNotch_04_T_W, 'WhiteNotch');
BlackSlot_01_FeaturePoint = getFeaturePointfromMarker(BlackSlot_01_T_W, 'BlackSlot');
BlackSlot_03_FeaturePoint = getFeaturePointfromMarker(BlackSlot_03_T_W, 'BlackSlot');
BlackSlot_05_FeaturePoint = getFeaturePointfromMarker(BlackSlot_05_T_W, 'BlackSlot');

% warp the trajectory based on feature points
LTT_Data_WhiteNotch_02 = LTT_Warping(LTT_Data_WhiteNotch, WhiteNotch_02_FeaturePoint, WhiteNotch_02_T_W, si);
LTT_Data_WhiteNotch_04 = LTT_Warping(LTT_Data_WhiteNotch, WhiteNotch_04_FeaturePoint, WhiteNotch_04_T_W, si);
LTT_Data_BlackSlot_01 = LTT_Warping(LTT_Data_BlackSlot, BlackSlot_01_FeaturePoint, BlackSlot_01_T_W, si);
LTT_Data_BlackSlot_03 = LTT_Warping(LTT_Data_BlackSlot, BlackSlot_03_FeaturePoint, BlackSlot_03_T_W, si);
LTT_Data_BlackSlot_05 = LTT_Warping(LTT_Data_BlackSlot, BlackSlot_05_FeaturePoint, BlackSlot_05_T_W, si);
LTT_Data_BlackSlot_01_End = LTT_Warping(LTT_Data_BlackSlot_End, BlackSlot_01_FeaturePoint, BlackSlot_01_T_W, si);
LTT_Data_ApproachTaping_03 = LTT_Warping(LTT_Data_ApproachTaping, BlackSlot_03_FeaturePoint, BlackSlot_03_T_W, si);

%%
%%%%%%%%%%%%%%%%%%%%%%%%%% Further Warp for Robot 2 Grapsing  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ropeFeaturePoint during teaching
rope_LeftEndPoint = LTT_Data_BlackSlot.FeaturePoint_xyz_W(:,8) + [0; -0.20; -0.03]; % rope left end is in the 20cm left side of current BlackSlot. 3cm below the BlackSlot surface
rope_RightEndPoint = LTT_Data_BlackSlot.TCP_T_W{1}(1:3,4,1) + [0;0;0.04];  % rope right end is fixed by R1 gripper. TCP + 4cm in z direction
Old_rope_FeaturePoint = getRopeFeaturePoint(rope_LeftEndPoint, rope_RightEndPoint);

%% Grasp between WhiteNotch_04, BlackSlot_05
rope_LeftEndPoint = LTT_Data_WhiteNotch_04.FeaturePoint_xyz_W(:,1); % rope left end is in right side of previous BlackSlot. 3cm below the BlackSlot surface
rope_RightEndPoint = LTT_Data_BlackSlot_05.TCP_T_W{1}(1:3,4,1) + [0;0;0.04]; % rope right end is fixed by R1 gripper. TCP + 4cm in z direction
New_rope_FeaturePoint = getRopeFeaturePoint(rope_LeftEndPoint, rope_RightEndPoint);

% warp robot2 movement
robot_idx = 2;
LTT_Data_BlackSlot_R2 = LTT_Warping(LTT_Data_BlackSlot, New_rope_FeaturePoint, BlackSlot_05_T_W, si, Old_rope_FeaturePoint, robot_idx);
% update Robot 2 graping movement into LTT_Data_BlackSlot_05
LTT_Data_BlackSlot_05.DesJntPos{2}(1:2,:) = LTT_Data_BlackSlot_R2.DesJntPos{2}(1:2,:);
LTT_Data_BlackSlot_05.TCP_T_W{2}(:,:,1:2) = LTT_Data_BlackSlot_R2.TCP_T_W{2}(:,:,1:2);
LTT_Data_BlackSlot_05.TCP_T_B{2}(:,:,1:2) = LTT_Data_BlackSlot_R2.TCP_T_B{2}(:,:,1:2);
LTT_Data_BlackSlot_05.TCP_xyzwpr_W{2}(1:2,:) = LTT_Data_BlackSlot_R2.TCP_xyzwpr_W{2}(1:2,:);
LTT_Data_BlackSlot_05.TCP_xyzwpr_B{2}(1:2,:) = LTT_Data_BlackSlot_R2.TCP_xyzwpr_B{2}(1:2,:);

%% Grasp between BlackSlot_05, BlackSlot_03
rope_LeftEndPoint = LTT_Data_BlackSlot_05.FeaturePoint_xyz_W(:,1) + [0;0;-0.03];
rope_RightEndPoint = LTT_Data_BlackSlot_03.TCP_T_W{1}(1:3,4,1) + [0;0;0.04];
New_rope_FeaturePoint = getRopeFeaturePoint(rope_LeftEndPoint, rope_RightEndPoint);

% warp robot2 movement
robot_idx = 2;
LTT_Data_BlackSlot_R2 = LTT_Warping(LTT_Data_BlackSlot, New_rope_FeaturePoint, BlackSlot_03_T_W, si, Old_rope_FeaturePoint, robot_idx);
% update Robot 2 graping movement into LTT_Data_BlackSlot_03
LTT_Data_BlackSlot_03.DesJntPos{2}(1:2,:) = LTT_Data_BlackSlot_R2.DesJntPos{2}(1:2,:);
LTT_Data_BlackSlot_03.TCP_T_W{2}(:,:,1:2) = LTT_Data_BlackSlot_R2.TCP_T_W{2}(:,:,1:2);
LTT_Data_BlackSlot_03.TCP_T_B{2}(:,:,1:2) = LTT_Data_BlackSlot_R2.TCP_T_B{2}(:,:,1:2);
LTT_Data_BlackSlot_03.TCP_xyzwpr_W{2}(1:2,:) = LTT_Data_BlackSlot_R2.TCP_xyzwpr_W{2}(1:2,:);
LTT_Data_BlackSlot_03.TCP_xyzwpr_B{2}(1:2,:) = LTT_Data_BlackSlot_R2.TCP_xyzwpr_B{2}(1:2,:);

%% Grasp between BlackSlot_03, BlackSlot_01
rope_LeftEndPoint = LTT_Data_BlackSlot_03.FeaturePoint_xyz_W(:,1) + [0;0;-0.03];
rope_RightEndPoint = LTT_Data_BlackSlot_01.TCP_T_W{1}(1:3,4,1) + [0;0;0.04];
New_rope_FeaturePoint = getRopeFeaturePoint(rope_LeftEndPoint, rope_RightEndPoint);

% warp robot2 movement
robot_idx = 2;
LTT_Data_BlackSlot_R2 = LTT_Warping(LTT_Data_BlackSlot, New_rope_FeaturePoint, BlackSlot_01_T_W, si, Old_rope_FeaturePoint, robot_idx);
% update Robot 2 graping movement into LTT_Data_BlackSlot_03
LTT_Data_BlackSlot_01.DesJntPos{2}(1:2,:) = LTT_Data_BlackSlot_R2.DesJntPos{2}(1:2,:);
LTT_Data_BlackSlot_01.TCP_T_W{2}(:,:,1:2) = LTT_Data_BlackSlot_R2.TCP_T_W{2}(:,:,1:2);
LTT_Data_BlackSlot_01.TCP_T_B{2}(:,:,1:2) = LTT_Data_BlackSlot_R2.TCP_T_B{2}(:,:,1:2);
LTT_Data_BlackSlot_01.TCP_xyzwpr_W{2}(1:2,:) = LTT_Data_BlackSlot_R2.TCP_xyzwpr_W{2}(1:2,:);
LTT_Data_BlackSlot_01.TCP_xyzwpr_B{2}(1:2,:) = LTT_Data_BlackSlot_R2.TCP_xyzwpr_B{2}(1:2,:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Orangle Cable trajectory 
% orange cable goes through WhiteNotch_04, BlackSlot_05, BlackSlot_03, BlackSlot_01
intervalTime = 1.5;
LTT_Data_Orange = combineMotion(LTT_Data_WhiteNotch_04,LTT_Data_BlackSlot_05,intervalTime);
LTT_Data_Orange = combineMotion(LTT_Data_Orange,LTT_Data_BlackSlot_03,  intervalTime);
LTT_Data_Orange = combineMotion(LTT_Data_Orange,LTT_Data_BlackSlot_01,  intervalTime);
% save Orange cable traj data for the use of CFS optimization
save('E:\Google Drive\FANUC_LTTData\beforeOpt\OrangeLine.mat', 'LTT_Data_WhiteNotch_04', 'LTT_Data_BlackSlot_05', ...
                                                     'LTT_Data_BlackSlot_03', 'LTT_Data_BlackSlot_01');

%% Green Cable Trajectory 
% green cable goes through WhiteNotch_02, BlackSlot_01
LTT_Data_Green = combineMotion(LTT_Data_WhiteNotch_02, LTT_Data_BlackSlot_01_End, 2);
% save Orange cable traj data for the use of CFS optimization
save('E:\Google Drive\FANUC_LTTData\beforeOpt\GreenLine.mat', 'LTT_Data_WhiteNotch_02', 'LTT_Data_BlackSlot_01_End');                                                 

%% Remember to check collision if workpiece location is chaged substantially!
disp('======================================================================')
input('Use CFS toolbox to check collision if the workpiece location changes substantially. Press any key to continue...')

%% Download Orange Cable Trajectory
Traj_Download_Run(LTT_Data_Orange, si, 'Download', 'noRun');

%% release brake
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(si.ParamSgnID, 'off');
tg = SimulinkRealTime.target;

%% Go Back Home Position and Open Gripper
opt = {}; opt.is_dual_arm = true; opt.robot_idx = [1,2];
PosCmd = [+50,0,0,0,-90,0; -70,0,0,0,-90,0]; 
tp_pos_run(si, PosCmd, opt)

gripper_action(si.ParamSgnID, 'DRIVE', cellstr(repmat('fully open', numel(si.ParamSgnID), 1))); 

%% Pick Orange Cable Connector
imThOpt = {}; imThOpt.type = 'orange'; imThOpt.plot_enb = 1;
opt = {}; opt.is_dual_arm = true; opt.robot_idx = 2;
pickConnector(imThOpt, opt, si)

%% Run Orange Cable Routing Traj
Traj_Download_Run(LTT_Data_Orange, si, 'noDownload', 'Run');

%% Open Robot 1 Gripper and Lift up
gripper_action(si.ParamSgnID{1}, 'DRIVE', cellstr(repmat('fully open', numel(si.ParamSgnID{1}), 1))); 
opt = {}; opt.is_dual_arm = true; opt.frame = 'world'; opt.robot_idx = 1;
offset = [0, 0, 150, 0, 0, 0];          % move robot 1 along world frame Z axis, 150mm to get more free space
relative_move(si, offset, [], opt);

%% Robot 2 Move to Open Area to Take Better Picture
opt = {}; opt.is_dual_arm = true; opt.robot_idx = 2;
PosCmd = [-60,0,0,0,-90,0]; 
tp_pos_run(si, PosCmd, opt)

%% Download Green Cable Trajectory
Traj_Download_Run(LTT_Data_Green, si, 'Download', 'noRun');

%% Make Sure Green Cable is on the Table 
disp('======================================================================')
input('Pick Green Cable. Please make sure Green Cable is on the table! Press any key to continue...')

%% Pick Green Cable Connector
imThOpt = {}; imThOpt.type = 'green'; imThOpt.plot_enb = 1;
opt = {}; opt.is_dual_arm = true; opt.robot_idx = 2;
pickConnector(imThOpt, opt, si)

%% Run Green Cable Trajectory
Traj_Download_Run(LTT_Data_Green, si, 'noDownload', 'Run');

%% Open Robot 1 Gripper and Lift Up
gripper_action(si.ParamSgnID{1}, 'DRIVE', cellstr(repmat('fully open', numel(si.ParamSgnID{1}), 1))); 
opt = {}; opt.is_dual_arm = true; opt.frame = 'world'; opt.robot_idx = 1;
offset = [0, 0, 150, 0, 0, 0];    % move robot 1 along world frame Z axis, 150mm to get more free space
relative_move(si, offset, [], opt);

%% Wait Human User to Put on the Black Plug
disp('======================================================================')
input('Please put on the black plug to lock the two cables! Press any key to continue...')

%% Grasp Taping Tool
TapingTool_T_W = ArucoMarker.T_w{ArucoMarker.ids == 0};
opt = {}; opt.is_dual_arm = true; opt.robot_idx = 2;
pickTapingTool(TapingTool_T_W, opt, si)

%% Approach to Taping Position
gripper_action(si.ParamSgnID{1}, 'DRIVE', cellstr(repmat('fully open', numel(si.ParamSgnID{1}), 1))); 
Traj_Download_Run(LTT_Data_ApproachTaping_03, si, 'Download', 'Run');

%% Robot 2 Taping
gripper_action(si.ParamSgnID{1}, 'DRIVE', cellstr(repmat('fully closed', numel(si.ParamSgnID{1}), 1))); 
pause(1);
Tape_Circling(si);

%% Decompose Tape_Cutting into Retreat, Approach to Button, Press for the ease of optimization

%% Retreat robot 1 to open area 
gripper_action(si.ParamSgnID{1}, 'DRIVE', cellstr(repmat('fully open', numel(si.ParamSgnID{1}), 1))); 
gripper_action(si.ParamSgnID{1}, 'DRIVE', cellstr(repmat('fully open', numel(si.ParamSgnID{1}), 1))); 
pause(1);
Tape_Cutting_Retreat(si)

%% Calculate the Points to Approach to Button
[qVec, tVec, rbt_pos] = Tape_Cutting_ApproachButton(si);
save('E:\Google Drive\FANUC_LTTData\beforeOpt\TapeCuttingApproach.mat', 'qVec', 'tVec', 'rbt_pos');

%% Choose to Run Human Defined Traj or Optimized Traj
disp('======================================================================')
UserSelection = input('Input 1 to run Human Defined Traj, 2 for Optimized Traj, or any other number to put back tool: ');

if UserSelection == 1
    
    % Use Human Defined Traj for approaching
    opt = {}; opt.is_dual_arm = true; opt.robot_idx = 1; opt.interpType = 'tcp';
    extcmd = ExtCmdGenerate(qVec, tVec, si, si.ri{opt.robot_idx}, [], opt);
    ExtCmdDownload(si.ParamSgnID{opt.robot_idx}, extcmd);
    tp_prg_run(si, opt);
    % press button
    Tape_Cutting_PressButton(si)
    % Use Human Define Traj for return
    opt.robot_idx = 1;
    rbt_pos = get_rbt_cur_pos(si, opt);
    qVec_return = [rbt_pos.jnt; qVec(3,:); qVec(2,:)];
    tVec = [2, 6];
    opt.interpType = 'tcp';
    extcmd = ExtCmdGenerate(qVec_return, tVec, si, si.ri{opt.robot_idx}, [], opt);
    ExtCmdDownload(si.ParamSgnID{opt.robot_idx}, extcmd);
    tp_prg_run(si, opt);
    
elseif UserSelection == 2
    
    % Use Optimized Traj for approaching
    load('E:\Google Drive\FANUC_LTTData\afterOpt\ExtCmd-PressTapingTool.mat')
    runExtCmd(ExtCmd, si)
    
    % press button
    Tape_Cutting_PressButton(si)
    
    % Use reversed Optimized Traj for return
    opt.robot_idx = 1;
    rbt_pos = get_rbt_cur_pos(si, opt);
    ExtCmd2 = ExtCmd;
    ExtCmd2{1}.vcmd_t(1:ExtCmd{1}.vcmd_EndCnt_t ,:) = flip(- ExtCmd2{1}.vcmd_t(1:ExtCmd{1}.vcmd_EndCnt_t ,:),1);
    ExtCmd2{1}.inipos_t = rbt_pos.jnt;
    runExtCmd(ExtCmd2, si)
    
end

%% Robot 2 Put Back Taping Tool
TapingTool_T_W = ArucoMarker.T_w{ArucoMarker.ids == 0};
opt = {}; opt.is_dual_arm = true; opt.robot_idx = 2;
putbackTapingTool(TapingTool_T_W, opt, si)

%% Go Back Home Position
opt = {}; opt.is_dual_arm = true; opt.robot_idx = [1,2];
PosCmd = [+50,0,0,0,-90,0; ...
          -70,0,0,0,-90,0]; 
tp_pos_run(si, PosCmd, opt)
gripper_action(si.ParamSgnID, 'DRIVE', cellstr(repmat('fully open', numel(si.ParamSgnID), 1))); 

%% Brake On
brake_on_off(si.ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);

