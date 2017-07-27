%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Load and Process BlackSlot Motion Primitive
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function LTT_Data_RopeKnot = Load_LTT_RopeKnot1(si)

% log data from file
opt.DataMode = 0;     % 0 - use previous LTT data to replay (default); 1 - get new LTT data but no replay
%opt.FileName = 'LTT_Data_20160915154147.mat';     % 0 - no specify file; 'filename str'
opt.FileName = 'LTT_Data_20170416235309.mat';     % 0 - no specify file; 'filename str'
LTT_Data = Load_LTT_Data(si, opt);

% remove 0.01s gripper waiting time
for i = 1:2
    index = find(LTT_Data.ReplayTime{i} < 0.05);
    LTT_Data.ReplayTime{i}(index+1) = LTT_Data.ReplayTime{i}(index+1) + LTT_Data.ReplayTime{i}(index);
    LTT_Data.ReplayTime{i}(index) = [];
    LTT_Data.DesJntPos{i}(index+1,:) = [];
    LTT_Data.GrpCmd{i}(index+1,:) = [];
end

% adjust speed of BlackSlot motion
LTT_Data.ReplayTime{1} = [2; 2; 2; 2; 2; 2];
LTT_Data.ReplayTime{2} = LTT_Data.ReplayTime{1};

% calculate TCP transformation matrix
for i = 1:2
    [TCP_xyzwpr_B, TCP_T_B] = fanucfkine(LTT_Data.DesJntPos{i}, si.ri{i});
    TCP_T_W = [];
    TCP_xyzwpr_W = [];
    for j = 1:size(TCP_T_B,3)
        TCP_T_W(:,:,j) = FrameTransform(TCP_T_B(:,:,j), 'T', 'B2W', si, i);
        TCP_xyzwpr_W(j,:) = T2xyzwpr(TCP_T_W(:,:,j));
    end
    LTT_Data.TCP_xyzwpr_B{i} = TCP_xyzwpr_B;
    LTT_Data.TCP_xyzwpr_W{i} = TCP_xyzwpr_W;
    LTT_Data.TCP_T_B{i} = TCP_T_B;
    LTT_Data.TCP_T_W{i} = TCP_T_W;
end



%% output
LTT_Data_RopeKnot = LTT_Data;

end
