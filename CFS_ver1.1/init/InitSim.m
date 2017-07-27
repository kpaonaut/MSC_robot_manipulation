%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%       Initialize the simulation parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%  Based on Wenjie's work in FANUC Robot Experimentor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  
%  Similar to the experimentor's init. 
%  Please update the parameters if needed
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization
siSim.robot_num = 2;
riSim = cell(siSim.robot_num,1);

%% Robot Number
riSim{1}.robot_no = 1;
riSim{2}.robot_no = 2;

%% Hardware On/Off State
siSim.wasStopped = 1;          % default: stopped 
riSim{1}.wasBrakeOff = 0;      % default: brake on
riSim{2}.wasBrakeOff = 0;      % default: brake on

%% Robot Master Count Parameter 
% Set the robot master count parameter in R-30iB robot control system.
% The robot master count represents the encorder value at the zero posture of the robot. 
% Each Single Robot has its own MasterCount !!!!!!!!!!!!!!!!!!

% % UCB LRMate200iD/7L No.1
% ri{1}.MasterCount = [-111678 -6944731 8489722 340004 -203572 -3755901];
% 
% % UCB LRMate200iD/7L No.2
% ri{2}.MasterCount = [105567 -6642288 9718216 193694 27427 -1010505]; 

% FANUC Internship  LRMate200iD/7L No.1 (D53965,R15201968,E15132888)
riSim{1}.MasterCount=[333096 -6773653 8926657 129130 -80980 -6290601];
% FANUC Internship  LR Mate 200iD/7L No.2 (D53966,R15201969,E15132889)
riSim{2}.MasterCount=[82449 -5809569 9136228 62648 -239540 -12282073];     

%% Common Parameters
siSim.AxisNum = 6;                 % number of robot axes
siSim.itp = 0.008;                 % itp time[s]
siSim.tsv = 0.001;                 % servo sampling time[s]
siSim.etsv = 0.00025;              % EtherCAT communication sampling time[s]
siSim.SolverTimeStep = 0.00025;    % Simuink solver fixed step time[s] 
siSim.ext_vel_max_cnt = 10000;     % itp number of external motion cmd 
siSim.nsv = siSim.itp/siSim.tsv;         % ratio of itp to tsv
siSim.FileScope_NumberOfSamples = 1000*30;     % 1000*A A[sec]     File scope record data number
siSim.UseAmp = [20 20 20 20 10 10];            % [Ap]              Amp size of R-30iB Mate Controller 
siSim.DSA_FlexFeedGear = [1 1 1 1 1 1]/10;     % DSA Setting Parameter - Flexible Feed Gear
siSim.timeGripperInputHold = 0.032;    % hold time for gripper input command (s) (should be at least 30ms)
siSim.gripper_des_mode = 3;        % desired gripper mode when dogin grasping (default: 3 - grab cable firmly)

%% Workspace Parameters (FANUC)
% % Testbed coordinates in design space frame
% siSim.UpperStagePos = [207.7   282     267;...
%                     440.9   470.8   267;...
%                     1196.1  -461.7  267;...
%                     963     -650.5  267]/1000;
% siSim.MidStagePos =   [49      153.5   215.5;...
%                     274.3   336     215.5;...
%                     1029.5  -596.6  215.5;...
%                     804.2   -779.1  215.5]/1000;               
%                 
% % Rotation angle RPY about the moving frame ZYX from world frame to design
% % space frame. This is the same rotation as the FANUC convetion WPR (about 
% % the fixed frame XYZ), with vector order revsered. Use rpy2r(RPY, 'zyx')
% % to compute the rotation matrix.
% siSim.RPY_W2DS = [-atan2(siSim.UpperStagePos(2,2)-siSim.UpperStagePos(1,2), siSim.UpperStagePos(2,1)-siSim.UpperStagePos(1,1)), 0, 0];
% siSim.WOrigin = mean(siSim.MidStagePos, 1);      % define world frame origin as the center of the middle stage
% siSim.WOrigin(3) = 0;                         % define world frame origin at the robot base stage
% siSim.T_DS2W = [rpy2r(siSim.RPY_W2DS, 'zyx') * [eye(3) -siSim.WOrigin']; 0 0 0 1];   % transformation from the design space frame to world frame
% 
% % Camera calibration center coordinates (this is for Ensenso camera)
% siSim.camera_type = 'ensenso';     % ensenso camera, currently kinect sensor is not calibrated
% siSim.k2 = [];                     % set k2 (kinect v2) object to empty
% siSim.CalibGridEdgeCenter = [705.46-27.74, -275.87-22.47, 267.5+1]/1000;
% siSim.CalibGridEdgeCenter_W = siSim.T_DS2W * [siSim.CalibGridEdgeCenter'; 1];
% siSim.CalibGridEdgeCenter_W = siSim.CalibGridEdgeCenter_W(1:3)';
% siSim.CalibGridCenter_W = siSim.CalibGridEdgeCenter_W + [0, 0.08, 0];
% 
% % Robot coordinates systems
% riSim{1}.BaseScrewPos = [0, 0, 0; 165, 0, 0; 165, -165, 0; 0, -165, 0]/1000;   % position coordinates of robot base screws in design space frame
% riSim{1}.BaseCenter = mean(riSim{1}.BaseScrewPos, 1);                             % center position of robot base
% riSim{1}.J1Origin = riSim{1}.BaseCenter + [0, 0, 0.330];          % robot J1 origin in design space frame
% riSim{1}.RPY_DS2B = [0, 0, 0];                                 % rotation angle RPY from design space frame to robot base (J1) frame
% riSim{1}.T_B2DS = [[rpy2r(riSim{1}.RPY_DS2B, 'zyx') riSim{1}.J1Origin']; 0 0 0 1];   % transformation from robot base (J1) frame to design space
% riSim{1}.T_B2W = siSim.T_DS2W * riSim{1}.T_B2DS;                     % transformation from robot base (J1) frame to world frame
% 
% riSim{2}.BaseScrewPos = [500, -700, 0; 665, -700, 0; 665, -865, 0; 500, -865, 0]/1000;  % position coordinates of robot base screws in design space frame
% riSim{2}.BaseCenter = mean(riSim{2}.BaseScrewPos, 1);             % center position of robot base
% riSim{2}.J1Origin = riSim{2}.BaseCenter + [0, 0, 0.330];          % robot J1 origin in design space frame
% riSim{2}.RPY_DS2B = [pi/2, 0, 0];                              % rotation angle RPY from design space frame to robot base (J1) frame
% riSim{2}.T_B2DS = [[rpy2r(riSim{2}.RPY_DS2B, 'zyx') riSim{2}.J1Origin']; 0 0 0 1];   % transformation from robot base (J1) frame to design space
% riSim{2}.T_B2W = siSim.T_DS2W * riSim{2}.T_B2DS;                     % transformation from robot base (J1) frame to world frame

%% Workspace Parameters (Berkeley)
% Rotation angle RPY about the moving frame ZYX from world frame to design
% space frame. This is the same rotation as the FANUC convetion WPR (about 
% the fixed frame XYZ), with vector order revsered. Use rpy2r(RPY, 'zyx')
% to compute the rotation matrix.
siSim.RPY_W2DS = [0, 0, 0];
siSim.WOrigin = [ -0.127, -0.717, 0];      % [-5, -21.25-7, 0] inch define world frame origin as the center of the middle stage

siSim.T_DS2W = [rpy2r(siSim.RPY_W2DS, 'zyx') * [eye(3) -siSim.WOrigin']; 0 0 0 1];   % transformation from the design space frame to world frame

% Camera calibration center coordinates (this is for Ensenso camera)
siSim.camera_type = 'ensenso';     % ensenso camera, currently kinect sensor is not calibrated
siSim.k2 = [];                     % set k2 (kinect v2) object to empty
siSim.CalibGridEdgeCenter = [705.46-27.74, -275.87-22.47, 267.5+1]/1000;
siSim.CalibGridEdgeCenter_W = siSim.T_DS2W * [siSim.CalibGridEdgeCenter'; 1];
siSim.CalibGridEdgeCenter_W = siSim.CalibGridEdgeCenter_W(1:3)';
siSim.CalibGridCenter_W = siSim.CalibGridEdgeCenter_W + [0, 0.08, 0];

% Robot coordinates systems
riSim{1}.BaseScrewPos = [0, 0, 0; 165, 0, 0; 165, -165, 0; 0, -165, 0]/1000;   % position coordinates of robot base screws in design space frame
riSim{1}.BaseCenter = mean(riSim{1}.BaseScrewPos, 1);                             % center position of robot base
riSim{1}.J1Origin = riSim{1}.BaseCenter + [0, 0, 0.330];          % robot J1 origin in design space frame
riSim{1}.RPY_DS2B = [0, 0, 0];                                 % rotation angle RPY from design space frame to robot base (J1) frame
riSim{1}.T_B2DS = [[rpy2r(riSim{1}.RPY_DS2B, 'zyx') riSim{1}.J1Origin']; 0 0 0 1];   % transformation from robot base (J1) frame to design space
riSim{1}.T_B2W = siSim.T_DS2W * riSim{1}.T_B2DS;                     % transformation from robot base (J1) frame to world frame

riSim{2}.BaseScrewPos = [0, -1269, 0; 165, -1269, 0; 165, -165 - 1269, 0; 0, -165 - 1269, 0]/1000;  % position coordinates of robot base screws in design space frame
riSim{2}.BaseCenter = mean(riSim{2}.BaseScrewPos, 1);             % center position of robot base
riSim{2}.J1Origin = riSim{2}.BaseCenter + [0, 0, 0.330];          % robot J1 origin in design space frame
riSim{2}.RPY_DS2B = [0, 0, 0];                              % rotation angle RPY from design space frame to robot base (J1) frame
riSim{2}.T_B2DS = [[rpy2r(riSim{2}.RPY_DS2B, 'zyx') riSim{2}.J1Origin']; 0 0 0 1];   % transformation from robot base (J1) frame to design space
riSim{2}.T_B2W = siSim.T_DS2W * riSim{2}.T_B2DS;                     % transformation from robot base (J1) frame to world frame

%% DH Parameters
% DH parameter for Robot 1
riSim{1}.n = siSim.AxisNum;       % robot axis number
riSim{1}.DH = ...   %  J1          J2          J3          J4          J5          J6 
             [      0           -pi/2       0           0           0           0;          % Theta0 (rad)
                    0           0           0           -0.420      0           -0.08;      % D (m)
                    0.050       0.440       0.035       0           0           0;          % A (m)
                    -pi/2       pi          -pi/2       pi/2        -pi/2       pi];        % alpha (rad)    
riSim{1}.J62Gripper = 0.027+0.0157+0.027;  % Adapter + F/T sensor + Adapter
riSim{1}.Gripper2TCP = 0.200-0.027;        % Gripper with Adapter - Adapter
riSim{1}.DH(:,end+1) = [0; riSim{1}.J62Gripper + riSim{1}.Gripper2TCP; 0; 0];    % DH for robot tool (gripper)
riSim{1}.DH(:,end+1) = [0;0;0;0];     % Workpiece DH ------------------------------------------------------------- To be changed
riSim{1}.DH(:,end+1) = [0;0.1107;0.2111;0];     % LTT Handle Bar  !! Relative to J6 not TCP!! Use DH(:,6) to calculate
% DH parameter for Robot 2
riSim{2}.n = siSim.AxisNum;       % robot axis number
riSim{2}.DH = ...   %  J1          J2          J3          J4          J5          J6 
             [      0           -pi/2       0           0           0           0;          % Theta0 (rad)
                    0           0           0           -0.420      0           -0.08;      % D (m)
                    0.050       0.440       0.035       0           0           0;          % A (m)
                    -pi/2       pi          -pi/2       pi/2        -pi/2       pi];        % alpha (rad)           
riSim{2}.J62Gripper = 0.027+0.0157+0.027;  % Adapter + F/T sensor + Adapter
riSim{2}.Gripper2TCP = 0.200-0.027;        % Gripper with Adapter - Adapter
riSim{2}.DH(:,end+1) = [0; riSim{2}.J62Gripper + riSim{2}.Gripper2TCP; 0; 0];    % DH for robot tool (gripper)
riSim{2}.DH(:,end+1) = [0;0;0;0];     % Workpiece DH ------------------------------------------------------------- To be changed
riSim{2}.DH(:,end+1) = [0;0.1107;0.2111;0];      % LTT Handle Bar  !! Relative to J6 not TCP!! Use DH(:,6) to calculate

%% Initial Position Setting
% This is to set robot dependent initial position. This will be used to
% reset initial position values using parameter tuning after model building
riSim{1}.iniJ1Off = 30;    % initial J1 offset for robot 1 (deg)
riSim{2}.iniJ1Off = -30;    % initial J1 offset for robot 2 (deg)
for rn = 1:siSim.robot_num
    riSim{rn}.iniJntPos = round([-(siSim.RPY_W2DS(1)+riSim{rn}.RPY_DS2B(1))/pi*180 + riSim{rn}.iniJ1Off, 0, 0, 0, -90, 0], 1);                    % initial joint position (deg)
    riSim{rn}.utool = [1 0 0 0; 0 1 0 0; 0 0 1 riSim{rn}.DH(2,riSim{rn}.n+1)*1000];    % UTOOL data (mm)
    [riSim{rn}.iniTcpPos, ~] = fanucfkine(riSim{rn}.iniJntPos, riSim{rn});
    riSim{rn}.iniTcpPos = round(riSim{rn}.iniTcpPos, 1);
    riSim{rn}.iniTcpPos_W = T2xyzwpr(riSim{rn}.T_B2W * xyzwpr2T(riSim{rn}.iniTcpPos));
end

% It seems realtime panel cannot tune the parameters if they are (part of) model argument.
% So have to use global variable for both robots
siSim.iniJntPos = riSim{1}.iniJntPos;                            % initial joint position (deg)
siSim.iniTcpPos = riSim{1}.iniTcpPos;

%% Servo Controller 
% Robot 1
% Postion Gain
riSim{1}.kv = [0.05 0.03 0.05 0.1 0.1 0.1];
% Integral Gain
riSim{1}.ki = [2 3 4 3 2 2];
% FF controller model (0: with gripper; 1: without gripper)
riSim{1}.model_no = 0;    

% Robot 2
% Postion Gain
riSim{2}.kv = [0.05 0.03 0.05 0.1 0.1 0.1];
% Integral Gain
riSim{2}.ki = [2 3 4 3 2 2];
% FF controller model (0: with gripper; 1: without gripper)
riSim{2}.model_no = 0;

%% Safety Check Parameter
% Safety Parameter for Robot 1
% Motor torque limit.  10000 represents amp max torque.
riSim{1}.SafetyTrqLimit = [3000 3000 3000 2000 2000 2000]; %[10000/AmpMax]
% Joint axis stroke limit [deg]
riSim{1}.rj_margin = ones(1,6)*2;
riSim{1}.rj_pos_limit_min = [-170, -100, -72, -190, -125, -360] + riSim{1}.rj_margin; %[deg]     
riSim{1}.rj_pos_limit_max = [170,  145,  240, 190, 125, 360] - riSim{1}.rj_margin; %[deg]
% Joint Velocity Limit [deg/s]
riSim{1}.velboundary_max = [35, 35, 50, 50, 50, 50];    % [deg/s]
% Boundary Collision 
riSim{1}.boundary_min = [-1.0, -1.0, -0.30]';     % x  y  z
riSim{1}.boundary_max = [1.0,  1.0,  100]';        % x  y  z
riSim{1}.tcp_boundary_min = riSim{1}.iniTcpPos_W(1:3)/1000 + [-0.25, -0.8, -0.18];     % x  y  z
riSim{1}.tcp_boundary_max = riSim{1}.iniTcpPos_W(1:3)/1000 + [0.4,  0.3,  0.4];        % x  y  z
% Self Collision 
riSim{1}.RADIUS = [0.13, 0.12, 0.06, 0.06, 0.05];
riSim{1}.BASE_Z = -riSim{1}.J1Origin(3);    
% Force Limit
riSim{1}.Force_limit = [50; 50; 50; 10; 10; 10];   % Force:N   Torque:Nm

% Safety Parameter for Robot 2
% Motor torque limit.  10000 represents amp max torque.
riSim{2}.SafetyTrqLimit = [3000 3000 3000 2000 2000 2000]; %[10000/AmpMax]
% Joint axis stroke limit [deg]
riSim{2}.rj_margin = ones(1,6)*2;
riSim{2}.rj_pos_limit_min = [-170, -100, -72, -190, -125, -360] + riSim{2}.rj_margin; %[deg]     
riSim{2}.rj_pos_limit_max = [170,  145,  240, 190, 125, 360] - riSim{2}.rj_margin; %[deg]
% Joint Velocity Limit [deg/s]
riSim{2}.velboundary_max = [35, 35, 50, 50, 50, 50];    % [deg/s]
% Boundary Collision 
riSim{2}.boundary_min = [-1.0, -1.0, -0.30]';     % x  y  z
riSim{2}.boundary_max = [1.0,  1.0,  100]';        % x  y  z
riSim{2}.tcp_boundary_min = riSim{2}.iniTcpPos_W(1:3)/1000 + [-0.25, -0.3, -0.18];     % x  y  z
riSim{2}.tcp_boundary_max = riSim{2}.iniTcpPos_W(1:3)/1000 + [0.4,  0.8,  0.4];        % x  y  z
% Self Collision 
riSim{2}.RADIUS = [0.13, 0.12, 0.06, 0.06, 0.05];
riSim{2}.BASE_Z = -riSim{2}.J1Origin(3);
% Force Limit
riSim{2}.Force_limit = [50; 50; 50; 10; 10; 10];   % Force:N   Torque:Nm

% Used in Dual arm coordination
siSim.DH = riSim{1}.DH;
siSim.RADIUS = riSim{1}.RADIUS;
siSim.BASE_Z = -0.330;  % Two robot base and radius are the same.
siSim.J1Origin  = [riSim{1}.J1Origin; riSim{2}.J1Origin];
siSim.T_B2W = zeros(4,4,2);
siSim.T_B2W(:,:,1) = riSim{1}.T_B2W;
siSim.T_B2W(:,:,2) = riSim{2}.T_B2W;

siSim.dstart = 0.150;   % 15cm
siSim.d0 = 0.050;       %  5cm

%% Robtic Toolbox model
siSim.rtb{1} = buildRTBmdl(riSim{1});
siSim.rtb{2} = buildRTBmdl(riSim{2});

%% Max Speed and Acceleration Time Constant of Internal Motion 
% Robot 1
% Internal Programed motion
    % Acceleration Time Constant
    riSim{1}.AccelTime1_s = [320 320 320 320 320 320]*2;   % [msec]
    riSim{1}.AccelTime2_s = riSim{1}.AccelTime1_s*0.5;           % [msec]
    % Max speed of joint axis 
    riSim{1}.jvelmax = [20 20 20 20 20 20];                % [deg/s]   
% Jog
    % Acceleration Time Constant
    riSim{1}.jog_AccelTime1_s = [96 96 96 96 96 96]*4;     % [msec]
    riSim{1}.jog_AccelTime2_s = riSim{1}.jog_AccelTime1_s*0.5;   % [msec]
    % Max speed of joint axis 
    riSim{1}.jog_jvelmax = [20 20 20 20 20 20];            % [deg/s]
% Hold stop
    % Acceleration Time Constant
    riSim{1}.ext_hold_ta1 = 100;                           % [itp]  
    riSim{1}.ext_hold_ta2 = 50;                            % [itp]

% Robot 2
% Internal Programed motion
    % Acceleration Time Constant
    riSim{2}.AccelTime1_s = [320 320 320 320 320 320]*2;   % [msec]
    riSim{2}.AccelTime2_s = riSim{2}.AccelTime1_s*0.5;           % [msec]
    % Max speed of joint axis 
    riSim{2}.jvelmax = [20 20 20 20 20 20];                % [deg/s]   
% Jog
    % Acceleration Time Constant
    riSim{2}.jog_AccelTime1_s = [96 96 96 96 96 96]*4;     % [msec]
    riSim{2}.jog_AccelTime2_s = riSim{2}.jog_AccelTime1_s*0.5;   % [msec]
    % Max speed of joint axis 
    riSim{2}.jog_jvelmax = [20 20 20 20 20 20];            % [deg/s]
% Hold stop
    % Acceleration Time Constant
    riSim{2}.ext_hold_ta1 = 100;                           % [itp]  
    riSim{2}.ext_hold_ta2 = 50;                            % [itp]
       
%% Mechanical Parameters
% Robot 1
% Motor torque constant
riSim{1}.TrqCons = [0.326 0.326 0.155 0.0818 0.0829 0.0829]; %[Nm/Ap]
riSim{1}.DSA2Ap = [20,20,20,20,10,10]/10000;   % [Ap/DSA]
% Gear reduction ratio
riSim{1}.Rratio = [4240/37 121/1 2960/29 3360/46 2000/24 1200/29];
% Plus direction of robot joint axis against motor plus direction.
% 1:same direction,   -1:opposite direction
riSim{1}.Motor2TP_sign = [1 -1 1 -1 1 -1];   
% Interaction Matrix
% robot motor pos [deg] --> robot joint axis [deg]
riSim{1}.interaction_J56 = 12500/3;   % J6=J5motor/r.pa.interaction_J56+J6motor/r.pa.Rratio(6)*r.pa.sign.Motor2TP_sign(6)
riSim{1}.interaction = diag(1./riSim{1}.Rratio.*riSim{1}.Motor2TP_sign);
riSim{1}.interaction(6,5) = 1/riSim{1}.interaction_J56;
% Inverse of Interaction Matrix
% robot joint axis[deg] --> Motor pos[2^20/rev]
riSim{1}.k_rbj2mpls = inv(riSim{1}.interaction)/360*2^20;
% Interaction Matrix2
% Motor pos[2^20/rev] -->robot joint axis[deg]
riSim{1}.k_mpls2rbj = inv(riSim{1}.k_rbj2mpls);
% Reduction vector
% Motor pos[2^20/rev] -->robot joint axis[deg]
% ri{1}.k_vec_mpls2rbj = diag(ri{1}.k_mpls2rbj)';

% Robot 2
% Motor torque constant
riSim{2}.TrqCons = [0.326 0.326 0.155 0.0818 0.0829 0.0829]; %[Nm/Ap]
riSim{2}.DSA2Ap = [20,20,20,20,10,10]/10000;   % [Ap/DSA]
% Gear reduction ratio
riSim{2}.Rratio = [4240/37 121/1 2960/29 3360/46 2000/24 1200/29];
% Plus direction of robot joint axis against motor plus direction.
% 1:same direction,   -1:opposite direction
riSim{2}.Motor2TP_sign = [1 -1 1 -1 1 -1];  
% Interaction Matrix
% robot motor pos [deg] --> robot joint axis [deg]
riSim{2}.interaction_J56 = 12500/3;   % J6=J5motor/r.pa.interaction_J56+J6motor/r.pa.Rratio(6)*r.pa.sign.Motor2TP_sign(6)
riSim{2}.interaction = diag(1./riSim{2}.Rratio.*riSim{2}.Motor2TP_sign);
riSim{2}.interaction(6,5) = 1/riSim{2}.interaction_J56;
% Inverse of Interaction Matrix
% robot joint axis[deg] --> Motor pos[2^20/rev]
riSim{2}.k_rbj2mpls = inv(riSim{2}.interaction)/360*2^20;
% Interaction Matrix2
%Motor pos[2^20/rev] -->robot joint axis[deg]
riSim{2}.k_mpls2rbj = inv(riSim{2}.k_rbj2mpls);
% Reduction vector
% Motor pos[2^20/rev] -->robot joint axis[deg]
% ri{2}.k_vec_mpls2rbj = diag(ri{2}.k_mpls2rbj)';

%% Initialization of Robot External Motion Command Variables 
% these data will be updated in init_tunable_params.m
siSim.extcmd.vcmd_t = zeros(siSim.ext_vel_max_cnt,siSim.AxisNum);
siSim.extcmd.vcmd_EndCnt_t = 1;
siSim.extcmd.inipos_t = siSim.iniJntPos;
siSim.extcmd.grippercmd = zeros(siSim.ext_vel_max_cnt,6);

%% 2nd Order Low Pass Filter (tsv time)
siSim.Filter.w0 = 2*pi*10;    % 10 Hz
siSim.Filter.D = 1;           % damping ratio
siSim.Filter.FilterC = tf(siSim.Filter.w0^2,[1,2*siSim.Filter.D*siSim.Filter.w0,siSim.Filter.w0^2]);
siSim.Filter.FilterD = c2d(siSim.Filter.FilterC,siSim.tsv);
siSim.Filter.num = cell2mat(siSim.Filter.FilterD.num);
siSim.Filter.den = cell2mat(siSim.Filter.FilterD.den);

% Lead Through Mode Filter (11/18/2015)
siSim.Filter2.w0 = 2*pi*2;    % 2 Hz
siSim.Filter2.D = 1;           % damping ratio
siSim.Filter2.FilterC = tf(siSim.Filter2.w0^2,[1,2*siSim.Filter2.D*siSim.Filter2.w0,siSim.Filter2.w0^2]);
siSim.Filter2.FilterD = c2d(siSim.Filter2.FilterC,siSim.tsv);
siSim.Filter2.num = cell2mat(siSim.Filter2.FilterD.num);
siSim.Filter2.den = cell2mat(siSim.Filter2.FilterD.den);

%% ATI Force Sensor
% % ATI mini 45 Force F/T Transducer (Berkeley FT16208)
% ri{1}.CalibrationMatrix = ...
%     [0.84694,-0.00013,2.25771,-47.38105,-2.19622,46.19011;
%     -2.49558,54.11911,2.28525,-27.38248,2.24670,-26.76265;
%     67.53655,4.50683,67.62002,1.77566,66.15999,3.66868;
%     -0.01158,0.37397,-1.06950,-0.21997,1.08646,-0.12505;
%     1.23392,0.08344,-0.66049,0.30482,-0.62385,-0.35227;
%     0.00477,-0.68678,0.06877,-0.70011,0.03601,-0.69183];
% 
% % Force calibrate for gripper + peg 
% ri{1}.ATI_Offset = [-14.1800, -12.9896, -8.4350, -0.3141, 0.5917, 0.0506];   
% ri{1}.Payload = [0, 0, -12.7553];                                              
% ri{1}.PayloadPos = [-0.0008, -0.0030, 0.0686];  
% 
% % Force calibrate for handle bar + gripper + peg
% % ri{1}.ATI_Offset = [ -14.3435 , -13.9990,  -11.5496,  -0.3241  ,  0.5770  ,  0.0889];   
% % ri{1}.Payload = [ 0,    0,  -14.2305];                                              
% % ri{1}.PayloadPos = [ 0.0098 ,   0.0011   , 0.0619];  
% 
% 
% % ATI mini 45 Force F/T Transducer (Berkeley FT16527)
% ri{2}.CalibrationMatrix = ...
%     [0.39619,   0.46036,   5.26750, -47.95831,  -2.71853,  46.19573;
%     -5.63504,  54.59105,   2.44200, -27.62502,   1.59129, -26.81337;
% 	68.01913,   2.99949,  64.99354,   3.90244,  70.01034,   4.12923;
% 	-0.02278,   0.37506,  -1.05006,  -0.26016,   1.09543,  -0.11248;
% 	 1.26270,   0.06042,  -0.65976,   0.28875,  -0.60694,  -0.35841;
% 	 0.09026,  -0.69384,   0.07241,  -0.70365,   0.02764,  -0.69034];
% 
% % Calibrated on 11/18/2015 Robot No.2 with Handle bar and Gripper
% ri{2}.ATI_Offset = [-10.8419, 0.2165, -5.3853, -0.37817, 0.44585, 0.19223];   
% ri{2}.Payload = [0, 0, -14.07996];                                              
% ri{2}.PayloadPos = [0.0081059; -0.0011928; 0.061659]; 

siSim.FS_ID_On = 0;    % 1- do force sensor calibration; 0- not do calibration

% Parameters for LTT and Repulsive force
siSim.LTT_Gain_High = [0.025, 0.025, 0.025, 0.8, 0.5, 0.8];
siSim.LTT_Gain_Low = siSim.LTT_Gain_High / 3;   
siSim.Frep_Gain = [0.3, 0.3, 0.3];
siSim.LTT_DeadZone = [1.5, 1.5, 1.5, 0.1, 0.1, 0.1];                           % Dead zone for LTT force
siSim.LTT_Speed_Limit = [0.15, 0.15, 0.15, 20/180*pi, 20/180*pi, 20/180*pi];   % Speed limit for LTT (m/sec, rad/sec), to cut off excessive force to avoid unstable motions during sudden turn overs

% ATI mini45 FT 16528 
riSim{1}.CalibrationMatrix = ...
    [ -0.92043   0.36985   4.02006 -47.19512  -2.96703  46.19897;
      -2.69263  53.86225   2.30519 -27.08931   1.01137 -26.65430;
      69.50063   3.21069  66.04647   3.59355  71.10316   2.93019;
       0.01020   0.37508  -1.08697  -0.25507   1.12928  -0.12013;
       1.26524   0.06441  -0.64009   0.28516  -0.62063  -0.35072;
       0.05578  -0.68521   0.03369  -0.69990   0.07472  -0.68432];

% Force calibrate for gripper 
if siSim.FS_ID_On
    riSim{1}.ATI_Offset = zeros(1,6);   
    riSim{1}.PayloadG =  zeros(1,3);                                               % Gravity force of payload in robot base frame      
    riSim{1}.PayloadCG = zeros(1,3);                                               % CG of Payload in robot sensor frame                     
else
    riSim{1}.ATI_Offset = [-10.2727, -3.63819, -9.28917, 0.12171, 0.54384, 0.14255];
    riSim{1}.PayloadG = [0, 0, -15.8803];                                          % Gravity force of payload in robot base frame
    riSim{1}.PayloadCG = [0.017042, -0.0042235, 0.067413];                         % CG of Payload in robot sensor frame 
end
riSim{1}.L_ATI2TCP = [0, 0, 0.200];                                                    % Distance from ATI sensor to TCP in robot tool frame
riSim{1}.L_ATI2LTT = [0.035/2+0.0195+0.1916-0.0576, 0.010, 0.027+0.046-0.005];         % Distance from ATI sensor to LTT in robot tool frame

%  ATI mini45 FT 16920  
riSim{2}.CalibrationMatrix = ...
        [-0.39731   0.21281   0.64558 -48.00311  -0.98747  47.55857;
         -3.62355  55.03061  -0.85691 -27.68902   2.53746 -27.63402 ;
		 69.02575   3.87832  67.63319   2.70061  68.70450   3.07214 ;
	     -0.02531   0.37887  -1.11721  -0.24989   1.12558  -0.12644 ;
	      1.24972   0.07849  -0.64249   0.29210  -0.63084  -0.35578 ;
	      0.03474  -0.70423   0.03190  -0.70856   0.01353  -0.69910] ;  

% Force calibrate for gripper 
if siSim.FS_ID_On
    riSim{2}.ATI_Offset = zeros(1,6);   
    riSim{2}.PayloadG =  zeros(1,3);                                               % Gravity force of payload in robot base frame      
    riSim{2}.PayloadCG = zeros(1,3);                                               % CG of Payload in robot sensor frame                    
else
    riSim{2}.ATI_Offset = [-12.5092, 5.52408, -0.564324, 0.044929, 0.39076, 0.12932];
    riSim{2}.PayloadG =  [0, 0, -17.9381];                                         % Gravity force of payload in robot base frame
    riSim{2}.PayloadCG = [0.025009, -0.0038461, 0.060038];                         % CG of Payload in robot sensor frame 
end
riSim{2}.L_ATI2TCP = [0, 0, 0.200];                                                    % Distance from ATI sensor to TCP in robot tool frame
riSim{2}.L_ATI2LTT = [0.035/2+0.0195+0.1916-0.0576, -0.010, 0.027+0.046-0.005];        % Distance from ATI sensor to LTT in robot tool frame
