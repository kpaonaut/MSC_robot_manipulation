%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Load and Process WhiteNotch Motion Primitive
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function LTT_Data_WhiteNotch = Load_LTT_WhiteNotch(si)

% log data from file
opt.DataMode = 0;     % 0 - use previous LTT data to replay (default); 1 - get new LTT data but no replay
%opt.FileName = 'LTT_Data_20160915151563.mat';     % 0 - no specify file; 'filename str'
opt.FileName = 'LTT_Data_20160915151564.mat';     % 0 - no specify file; 'filename str'
LTT_Data = Load_LTT_Data(si, opt);

% remove 0.01s gripper waiting time
for i = 1:2
    index = find(LTT_Data.ReplayTime{i} < 0.05);
    LTT_Data.ReplayTime{i}(index+1) = LTT_Data.ReplayTime{i}(index+1) + LTT_Data.ReplayTime{i}(index);
    LTT_Data.ReplayTime{i}(index) = [];
    LTT_Data.DesJntPos{i}(index+1,:) = [];
    LTT_Data.GrpCmd{i}(index+1,:) = [];
end

% adjust speed of WhiteNotch motion primitive
LTT_Data.ReplayTime{1} = [2; 1; 7; 4; 2; 1.5];
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

% Marker transformation matrix during teaching
LTT_Data.Marker_T_W = [0.00515724953172059,-0.999986701300206,0,0.125004796311259; ...
                 0.999986701300206,0.00515724953172059,0,-0.395496390759945; ...
                 0,0,1,0.390785239636898; ...
                 0,0,0,1];
             
% generate feature points from Marker_T_W
LTT_Data.FeaturePoint_xyz_W = getFeaturePointfromMarker(LTT_Data.Marker_T_W, 'WhiteNotch');

%% output
LTT_Data_WhiteNotch = LTT_Data;

end
