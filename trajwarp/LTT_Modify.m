%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       *Test File* for Modifying LTT_Data Manually
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
robot_idx = 2;
point_idx = 2;

Data_ToBeModified = LTT_Data_BlackSlot;

%% Modify T_w Manually and Get the Corresponding Joint Angle
T_W = Data_ToBeModified.TCP_T_W{robot_idx}(:,:,point_idx)
T_W(1:3,4) = T_W(1:3,4) + [0; 0.0; +0.01]

T_B = FrameTransform(T_W, 'T', 'W2B', si, robot_idx);
xyzwpr_B = T2xyzwpr(T_B);
opt = {};
opt.is_dual_arm = true;
PosCmd = fanucikine(xyzwpr_B, si.ri{robot_idx}, Data_ToBeModified.DesJntPos{robot_idx}(point_idx,:) , opt)

%% Get Current Pos
rbt_pos = get_rbt_cur_pos(si)


%% Find the desired index in LTT Data
find(LTTData2(:,1)< - 32 )