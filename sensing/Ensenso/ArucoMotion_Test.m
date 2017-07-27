%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       *Test file* for Aruco detection and motion generation
%       Please run cell by cell, instead of running in once
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Open Ensenso
nxOpenCam

%% get xyz depth data and rgb data
[camData.xyz, camData.rgba] = nxGetPtCld();  
camData.xyz = permute(camData.xyz,[3,2,1]);
camData.rgba = permute(camData.rgba,[3,2,1]);

camData.rgb = nxGetRGBonly();  
camData.rgb = permute(camData.rgb,[3,2,1]);

% transform xyz data into World frame (m)
temp = reshape(camData.xyz/1000,1024*1280,3);
camData.xyz_w = reshape(temp + repmat(si.CalibGridCenter_W, 1024*1280, 1), 1024,1280,3);
% need add additional offset !!!  x: 6mm  y: -4mm

% detect ID and corners of Aruco marker
[ids, corners, image_corner] = detectArucoCorner(camData.rgb);


if ids == -1
    error('Error: Could not find any Aruco marker!')
end

% calculate Marker xyz and pose
ArucoMarker = ArucoProcess(double(ids), corners, image_corner , double(camData.xyz_w));

drawAruco(ArucoMarker)

%% Generate Motion to Grasp Taping Tool

%% Transform to Robot 2 base frame
interestedID = 2;
index = find(ArucoMarker.ids == interestedID);
T_W = ArucoMarker.T_w{index}
robot_idx = 2;
T_B = FrameTransform(T_W, 'T', 'W2B', si, robot_idx)
T_Hole_B =  T_B*[[rotx(pi),zeros(3,1)];[0,0,0,1]]    % flip the y,z direction of tcp
T_Hole_B = T_Hole_B* [[eye(3),[-0.046; +0.0055; +0.022]];[0,0,0,1]]  % Marker-Hole offset  

T_Hole_above_B =  T_Hole_B * [[eye(3),[0; 0; -0.10]];[0,0,0,1]] % above Hole 10cm   

opt.SI_tcp = 0;   % 0 - degree, mm (default); 1 - radian, m  
Hole_xyzwpr = T2xyzwpr(T_Hole_B, opt)
Hole_above_xyzwpr = T2xyzwpr(T_Hole_above_B, opt)

%% Brake Off
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(si.ParamSgnID, 'off');

%% Open Gripper
opt.robot_idx = 2;   % robot index set for which operations should be conducted (default: all robots)
ParamSgnID = si.ParamSgnID{opt.robot_idx};
gripper_action(ParamSgnID, 'DRIVE', cellstr(repmat('fully open', numel(ParamSgnID), 1))); 

%% move to Hole_above_xyzwpr
opt.robot_idx = 2;   % robot index set for which operations should be conducted (default: all robots)
opt.tcp = 1;         % 0 - joint position (default); 1 - tcp position
opt.SI_jnt = 0;      % 0 - degree (default); 1 - radian
opt.SI_tcp = 0;      % 0 - degree, mm (default); 1 - radian, m
tp_pos_run(si, Hole_above_xyzwpr, opt)

%% move to target Hole_xyzwpr
opt.robot_idx = 2;   % robot index set for which operations should be conducted (default: all robots)
opt.tcp = 1;         % 0 - joint position (default); 1 - tcp position
opt.SI_jnt = 0;      % 0 - degree (default); 1 - radian
opt.SI_tcp = 0;      % 0 - degree, mm (default); 1 - radian, m
tp_pos_run(si, Hole_xyzwpr, opt)


%% Close Gripper
gripper_action(ParamSgnID, 'DRIVE', cellstr(repmat('grab cable taping tool firmly', numel(ParamSgnID), 1))); 
setparam(tg, ParamSgnID.DES_MODE, gripper_mode('fully closed'));
setparam(tg, ParamSgnID.DES_MODE, gripper_mode('grab cable connector firmly'));


%%
pause(5)
gripper_action(ParamSgnID, 'DRIVE', cellstr(repmat('fully closed', numel(ParamSgnID), 1)));  
setparam(tg, si.ParamSgnID{1}.DES_MODE, gripper_mode('fully closed'));

gripper_action(si.ParamSgnID{1}, 'DRIVE', cellstr(repmat('fully open', numel(ParamSgnID), 1))); 
pause(5)
gripper_action(si.ParamSgnID{1}, 'DRIVE', cellstr(repmat('fully closed', numel(ParamSgnID), 1)));  


%% Open Gripper
gripper_action(ParamSgnID, 'DRIVE', cellstr(repmat('fully open', numel(ParamSgnID), 1))); 

%% move to Hole_above_xyzwpr
opt.robot_idx = 2;   % robot index set for which operations should be conducted (default: all robots)
opt.tcp = 1;         % 0 - joint position (default); 1 - tcp position
opt.SI_jnt = 0;      % 0 - degree (default); 1 - radian
opt.SI_tcp = 0;      % 0 - degree, mm (default); 1 - radian, m
tp_pos_run(si, Hole_above_xyzwpr, opt)


%% move to target joint
opt.robot_idx = 2;   % robot index set for which operations should be conducted (default: all robots)
opt.tcp = 0;         % 0 - joint position (default); 1 - tcp position
opt.SI_jnt = 0;      % 0 - degree (default); 1 - radian
opt.SI_tcp = 0;      % 0 - degree, mm (default); 1 - radian, m
tp_pos_run(si, [-60,0,0,0,-90,0],opt);

%% Brake On
brake_on_off(si.ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);
