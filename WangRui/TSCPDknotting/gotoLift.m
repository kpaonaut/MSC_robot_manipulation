%% This function adds a point on the path where the robot arm goes to a higher place before it grabs the rope.
% This is to ensure that the end effector does not grasp sideways, in which
% case the grasping is very likely to fail
% Rui Wang, 11/17/2017 MSC Lab, UC Berkeley
function LTT_Data_Test_tmp_new = gotoLift(LTT_Data_Test_tmp, si) % only deals with first robot lift
n = size(LTT_Data_Test_tmp.DesJntPos{1}, 1);
idx=1; % suppose it is the first robot manipulating. only lift it.

TCP_xyzwpr_W{idx} = LTT_Data_Test_tmp.TCP_xyzwpr_W{idx}(1, :); TCP_xyzwpr_W{idx}(1, 3) = 150; % lift up
TCP_T_W{idx} = xyzwpr2T(TCP_xyzwpr_W{idx});
TCP_T_B{idx} = FrameTransform(TCP_T_W{idx}, 'T', 'W2B', si, idx);
TCP_xyzwpr_B{idx} = T2xyzwpr(TCP_T_B{idx});
DesJntPos{idx} = fanucikine(TCP_xyzwpr_B{idx}, si.ri{idx}, LTT_Data_Test_tmp.DesJntPos{idx}(2, :));

TCP_xyzwpr_W{3-idx} = LTT_Data_Test_tmp.TCP_xyzwpr_W{3-idx}(1, :);% not lift up
TCP_T_W{3-idx} = xyzwpr2T(TCP_xyzwpr_W{3-idx});
TCP_T_B{3-idx} = FrameTransform(TCP_T_W{3-idx}, 'T', 'W2B', si, 3-idx);
TCP_xyzwpr_B{3-idx} = T2xyzwpr(TCP_T_B{3-idx});
DesJntPos{3-idx} = fanucikine(TCP_xyzwpr_B{3-idx}, si.ri{3-idx}, LTT_Data_Test_tmp.DesJntPos{3-idx}(1, :));

for idx = 1:2
    LTT_Data_Test_tmp.DesJntPos{idx}(2 : n+1, :) = LTT_Data_Test_tmp.DesJntPos{idx}(1:n, :);
    LTT_Data_Test_tmp.DesJntPos{idx}(1, :) = DesJntPos{idx};
    LTT_Data_Test_tmp.ReplayTime{idx} = [LTT_Data_Test_tmp.ReplayTime{idx}; LTT_Data_Test_tmp.ReplayTime{idx}(1)];% append one replay time
    
    LTT_Data_Test_tmp.GrpCmd{idx}(2 : n+1, :) = LTT_Data_Test_tmp.GrpCmd{idx}(1 : n, :);
    LTT_Data_Test_tmp.GrpCmd{idx}(1, :) = [0 0 0 0 0 1]; % grip keep!
end

LTT_Data_Test_tmp_new = LTT_Data_Test_tmp;