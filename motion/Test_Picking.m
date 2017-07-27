%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       Main File for Test Picking using Camera
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/08/26
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This is just a test to see if camera sensing is accurate enough for robot
% to pick up something

function Test_Picking(si, opt)

if nargin < 2,  opt = [];  end
if ~isfield(opt,'robot_idx') || isempty(opt.robot_idx),  opt.robot_idx = 1;  end   % robot index set for which operations should be conducted (default: robot 1)

rsi = si.ri{opt.robot_idx};
ParamSgnID = si.ParamSgnID{opt.robot_idx};
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(ParamSgnID, 'off');

init_pos_back(si, opt);

%% Get the position
imThOpt.type = 'orange';
imThOpt.plot_enb = 1;
camData = cam2pcl(imThOpt);

%region = [-inf, -inf, 250; 300, 450, 320]/1000;  % specify the region to part of the upper stage
region = [-200, -150, 220; 100, 200, 280]/1000;  % specify the region to part of the upper stage
camData = camPCL2World(si, camData, region, opt);

% find the end point with the largest y value
[~, max_idx] = max(camData.xyz_w(:,2));
endpoint_idx = abs(camData.xyz_w(:,2) - camData.xyz_w(max_idx,2)) < 0.01;
xyz_m_b = mean(camData.xyz_b(endpoint_idx, :));

opt.tcp = 1;
PosCmd = [xyz_m_b * 1000, 180, 0, 129];
PosCmd(3) = PosCmd(3) - 5;  % the camera only sees the top of the cable, need to adjust height offset
UpPosCmd = PosCmd;
UpPosCmd(3) = UpPosCmd(3) + 250;

%% Go and picking 
gripper_action(ParamSgnID, 'DRIVE', cellstr(repmat('fully open', numel(ParamSgnID), 1)));  
tp_pos_run(si, UpPosCmd, opt);
tp_pos_run(si, PosCmd, opt);
gripper_action(ParamSgnID, 'DRIVE', cellstr(repmat('grab cable firmly', numel(ParamSgnID), 1)));  
tp_pos_run(si, UpPosCmd, opt);
gripper_action(ParamSgnID, 'DRIVE', cellstr(repmat('fully open', numel(ParamSgnID), 1)));  

%% Stop the application
brake_on_off(ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);

end