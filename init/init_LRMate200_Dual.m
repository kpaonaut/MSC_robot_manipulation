%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Main File
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 06/18/2015       (Based on H.Nakagawa's Dual Robot Experimentor)
%  MSC Lab, U.C.Berkeley
%  Modified on 11/11/2015 by Te Tang
%  Modified on 05/20/2016 by Wenjie Chen for FANUC setups
%  Modified on 09/28/2016 by Te Tang (1. F/T sensor recalibrate 2. Enlarge vel safety margin 3. Increase Jog vel )
%  Modified on 04/13/2017 by Te Tang (For two LRMate200ID/7L Robots at Berkeley)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% If change Robot/Target PC/Force Sensor/EndTool, please change the following parameters:
% When changing the robot:
% 1. Modify the following parameters in init_LRMate200_Dual.m file:
% 	a). master count	b). gear ration		c) DH parameters	d) safety setting parameters
% 	e). PID controller gains
% 	f). torque constant, motion2TP_sign, interaction matrix
% 	g). motion generation parameters (when necessary)
% 2. Modify 'rlib/CMT_TRQ.c' for the kinematic and dynamic parameters and then re-mex the file
% 3. Modify forward and inverse kinematics mex functions (e.g., rlfklr2d7l.mexw64 and rliklr2d7l.mexw64). Ask FANUC to make this change.
% 
% When changing the force sensor/gripper/payload:
% 1. Modify the following parameters in init_LRMate200_Dual.m file:
% 	a). DH parameter related with the TCP
% 	b). Force sensor CalibrationMatrix parameters
% 	c). Relative positions between force sensor and LTT position/TCP ('L_ATI2TCP' and 'L_ATI2LTT')
% 	d). set 'si.FS_ID_On = 1' to do force sensor calibration
% 2. Modify 'gripper_mode.m' if you change the gripper mode setting
% 
% When changing the workspace structure:
% 1. Modify the following parameters in init_LRMate200_Dual.m file:
% 	a). workspace parameters
% 	b). safety setting parameters
% 	
% When changing the target PC:
% 1. Modify EtherCAT bus/slot number in the simulink file (e.g., LRMate200Experimentor_Dual.slx). Use 'target.getPCIInfo' to obtain the information. (currently for Misumi target PC [5, 9]; for Speedgoat target PC [6, 0];)
% 
% When changing the camera:
% 1. Calibrate the cameara(s) and its coordinates system using the calibration software (e.g., Ensenso NxView)
% 2. Update the calibrated parameters and camera type in the init_LRMate200_Dual.m file.

function si = init_LRMate200_Dual()

fprintf('Now starting FANUC Dual Robot Experimentor ...\n\n');

%% Initialization
si.robot_num = 2;
ri = cell(si.robot_num,1);

init_bus_objects;           % Initilize customized bus objects
% init_model_workspace;     % Not necessary now, will be auto executed by referenced model.
check_robot_model_copy;
unload_all_models;

%% Robot Number
ri{1}.robot_no = 1;
ri{2}.robot_no = 2;

%% Hardware On/Off State
si.wasStopped = 1;          % default: stopped 
ri{1}.wasBrakeOff = 0;      % default: brake on
ri{2}.wasBrakeOff = 0;      % default: brake on

%% Robot Master Count Parameter 
% Set the robot master count parameter in R-30iB robot control system.
% The robot master count represents the encorder value at the zero posture of the robot. 
% Each Single Robot has its own MasterCount !!!!!!!!!!!!!!!!!!

% % UCB LRMate200iD/7L No.1
ri{1}.MasterCount = [-1160254 8259621 1149690 -184284 3990732 -85885]; % MasterCount updated in 10/16/2016

% % UCB LRMate200iD/7L No.2
ri{2}.MasterCount = [105567 -350832 281032 -330594 551715 -486217];  % MasterCount updated in 10/16/2016

% FANUC Internship  LRMate200iD/7L No.1 (D53965,R15201968,E15132888)
% ri{1}.MasterCount=[333096 -6773653 8926657 129130 -80980 -6290601];
% FANUC Internship  LR Mate 200iD/7L No.2 (D53966,R15201969,E15132889)
% ri{2}.MasterCount=[82449 -5809569 9136228 62648 -239540 -12282073];     

%% Common Parameters
si.AxisNum = 6;                 % number of robot axes
si.itp = 0.008;                 % itp time[s]
si.tsv = 0.001;                 % servo sampling time[s]
si.etsv = 0.00025;              % EtherCAT communication sampling time[s]
si.SolverTimeStep = 0.00025;    % Simuink solver fixed step time[s] 
si.ext_vel_max_cnt = 10000;     % itp number of external motion cmd 
si.nsv = si.itp/si.tsv;         % ratio of itp to tsv
si.FileScope_NumberOfSamples = 1000*30;     % 1000*A A[sec]     File scope record data number
si.UseAmp = [20 20 20 20 10 10];            % [Ap]              Amp size of R-30iB Mate Controller 
si.DSA_FlexFeedGear = [1 1 1 1 1 1]/10;     % DSA Setting Parameter - Flexible Feed Gear
si.timeGripperInputHold = 0.032;    % hold time for gripper input command (s) (should be at least 30ms)
si.gripper_des_mode = 3;        % desired gripper mode when dogin grasping (default: 3 - grab cable firmly)

%% Workspace Parameters
                         
% Rotation angle RPY about the moving frame ZYX from world frame to design
% space frame. This is the same rotation as the FANUC convetion WPR (about 
% the fixed frame XYZ), with vector order revsered. Use rpy2r(RPY, 'zyx')
% to compute the rotation matrix.
si.RPY_W2DS = [0, 0, 0];
si.WOrigin = [ -0.127, -0.717, 0];      % [-5, -21.25-7, 0] inch define world frame origin as the center of the middle stage

si.T_DS2W = [rpy2r(si.RPY_W2DS, 'zyx') * [eye(3) -si.WOrigin']; 0 0 0 1];   % transformation from the design space frame to world frame

% Camera calibration center coordinates (this is for Ensenso camera)
si.camera_type = 'ensenso';     % ensenso camera, currently kinect sensor is not calibrated
si.k2 = [];                     % set k2 (kinect v2) object to empty
si.CalibGridEdgeCenter = [705.46-27.74, -275.87-22.47, 267.5+1]/1000;
si.CalibGridEdgeCenter_W = si.T_DS2W * [si.CalibGridEdgeCenter'; 1];
si.CalibGridEdgeCenter_W = si.CalibGridEdgeCenter_W(1:3)';
si.CalibGridCenter_W = si.CalibGridEdgeCenter_W + [0, 0.08, 0];

% Robot coordinates systems
ri{1}.BaseScrewPos = [0, 0, 0; 165, 0, 0; 165, -165, 0; 0, -165, 0]/1000;   % position coordinates of robot base screws in design space frame
ri{1}.BaseCenter = mean(ri{1}.BaseScrewPos, 1);                             % center position of robot base
ri{1}.J1Origin = ri{1}.BaseCenter + [0, 0, 0.330];          % robot J1 origin in design space frame
ri{1}.RPY_DS2B = [0, 0, 0];                                 % rotation angle RPY from design space frame to robot base (J1) frame
ri{1}.T_B2DS = [[rpy2r(ri{1}.RPY_DS2B, 'zyx') ri{1}.J1Origin']; 0 0 0 1];   % transformation from robot base (J1) frame to design space
ri{1}.T_B2W = si.T_DS2W * ri{1}.T_B2DS;                     % transformation from robot base (J1) frame to world frame

ri{2}.BaseScrewPos = [0, -1269, 0; 165, -1269, 0; 165, -165 - 1269, 0; 0, -165 - 1269, 0]/1000;  % position coordinates of robot base screws in design space frame
ri{2}.BaseCenter = mean(ri{2}.BaseScrewPos, 1);             % center position of robot base
ri{2}.J1Origin = ri{2}.BaseCenter + [0, 0, 0.330];          % robot J1 origin in design space frame
ri{2}.RPY_DS2B = [0, 0, 0];                              % rotation angle RPY from design space frame to robot base (J1) frame
ri{2}.T_B2DS = [[rpy2r(ri{2}.RPY_DS2B, 'zyx') ri{2}.J1Origin']; 0 0 0 1];   % transformation from robot base (J1) frame to design space
ri{2}.T_B2W = si.T_DS2W * ri{2}.T_B2DS;                     % transformation from robot base (J1) frame to world frame

%% DH Parameters
% DH parameter for Robot 1
ri{1}.n = si.AxisNum;       % robot axis number
ri{1}.DH = ...   %  J1          J2          J3          J4          J5          J6 
             [      0           -pi/2       0           0           0           0;          % Theta0 (rad)
                    0           0           0           -0.420      0           -0.08;      % D (m)
                    0.050       0.440       0.035       0           0           0;          % A (m)
                    -pi/2       pi          -pi/2       pi/2        -pi/2       pi];        % alpha (rad)    
ri{1}.J62Gripper = 0.027+0.0157+0.027;  % Adapter1 + F/T sensor + Adapter2
ri{1}.Gripper2TCP = 0.1714; %0.200-0.027;        % Gripper with Adapter - Adapter
ri{1}.DH(:,end+1) = [0; ri{1}.J62Gripper + ri{1}.Gripper2TCP; 0; 0];    % DH for robot tool (gripper)
%ri{1}.DH(:,end+1) = [0;0.1143;0.206;0];     % Workpiece DH ------------------------------------------------------------- To be changed
ri{1}.DH(:,end+1) = [0;0;0;0];     % Workpiece DH

% DH parameter for Robot 2
ri{2}.n = si.AxisNum;       % robot axis number
ri{2}.DH = ...   %  J1          J2          J3          J4          J5          J6 
             [      0           -pi/2       0           0           0           0;          % Theta0 (rad)
                    0           0           0           -0.420      0           -0.08;      % D (m)
                    0.050       0.440       0.035       0           0           0;          % A (m)
                    -pi/2       pi          -pi/2       pi/2        -pi/2       pi];        % alpha (rad)           
ri{2}.J62Gripper = 0.027+0.0157+0.027;  % Adapter + F/T sensor + Adapter
ri{2}.Gripper2TCP = 0.14605; %0.200-0.027;        % Gripper with Adapter - Adapter
ri{2}.DH(:,end+1) = [0; ri{2}.J62Gripper + ri{2}.Gripper2TCP; 0; 0];    % DH for robot tool (gripper)
%ri{2}.DH(:,end+1) = [0;0.1143;0.206;0];     % Workpiece DH ------------------------------------------------------------- To be changed
ri{2}.DH(:,end+1) = [0;0;0;0]; 

%% Initial Position Setting
% This is to set robot dependent initial position. This will be used to
% reset initial position values using parameter tuning after model building
ri{1}.iniJ1Off = 0;    % initial J1 offset for robot 1 (deg)         
ri{2}.iniJ1Off = 0;    % initial J1 offset for robot 2 (deg)
for rn = 1:si.robot_num
    ri{rn}.iniJntPos = round([-(si.RPY_W2DS(1)+ri{rn}.RPY_DS2B(1))/pi*180 + ri{rn}.iniJ1Off, 0, 0, 0, -90, 0], 1);                    % initial joint position (deg)
    ri{rn}.utool = [1 0 0 0; 0 1 0 0; 0 0 1 ri{rn}.DH(2,ri{rn}.n+1)*1000];    % UTOOL data (mm)
    [ri{rn}.iniTcpPos, ~] = fanucfkine(ri{rn}.iniJntPos, ri{rn});
    ri{rn}.iniTcpPos = round(ri{rn}.iniTcpPos, 1);
    ri{rn}.iniTcpPos_W = T2xyzwpr(ri{rn}.T_B2W * xyzwpr2T(ri{rn}.iniTcpPos));
end

% It seems realtime panel cannot tune the parameters if they are (part of) model argument.
% So have to use global variable for both robots
si.iniJntPos = ri{1}.iniJntPos;                            % initial joint position (deg)
si.iniTcpPos = ri{1}.iniTcpPos;

%% Servo Controller 
% Robot 1
% Postion Gain
ri{1}.kv = [0.05 0.03 0.05 0.1 0.1 0.1];
% Integral Gain
ri{1}.ki = [2 3 4 3 2 2];
% FF controller model (0: with gripper; 1: without gripper)
ri{1}.model_no = 0;    

% Robot 2
% Postion Gain
ri{2}.kv = [0.05 0.03 0.05 0.1 0.1 0.1];
% Integral Gain
ri{2}.ki = [2 3 4 3 2 2];
% FF controller model (0: with gripper; 1: without gripper)
ri{2}.model_no = 0;

%% Safety Check Parameter
% Safety Parameter for Robot 1
% Motor torque limit.  10000 represents amp max torque.
ri{1}.SafetyTrqLimit = [3000 3000 3000 2000 2000 2000]; %[10000/AmpMax]
% Joint axis stroke limit [deg]
ri{1}.rj_margin = ones(1,6)*2;
ri{1}.rj_pos_limit_min = [-170, -100, -72, -190, -125, -360] + ri{1}.rj_margin; %[deg]     
ri{1}.rj_pos_limit_max = [170,  145,  240, 190, 125, 360] - ri{1}.rj_margin; %[deg]
% Joint Velocity Limit [deg/s]
%ri{1}.velboundary_max = [35, 35, 50, 50, 50, 50];    % [deg/s]
ri{1}.velboundary_max = 1/4*[370,310, 410, 550, 545, 1000];   % [deg/s]


% Boundary Collision 
ri{1}.boundary_min = [-1.0, -1.0, -0.30]';     % x  y  z
ri{1}.boundary_max = [1.0,  1.0,  100]';        % x  y  z
ri{1}.tcp_boundary_min = ri{1}.iniTcpPos_W(1:3)/1000 + [-0.25, -0.8-0.2, -0.18 - 0.02];     % x  y  z
ri{1}.tcp_boundary_max = ri{1}.iniTcpPos_W(1:3)/1000 + [0.4,  0.3,  0.4];        % x  y  z
% Self Collision 
ri{1}.RADIUS = [0.13, 0.12, 0.06, 0.06, 0.05];
ri{1}.BASE_Z = -ri{1}.J1Origin(3);    
% Force Limit
ri{1}.Force_limit = [50; 50; 50; 10; 10; 10];   % Force:N   Torque:Nm

% Safety Parameter for Robot 2
% Motor torque limit.  10000 represents amp max torque.
ri{2}.SafetyTrqLimit = [3000 3000 3000 2000 2000 2000]; %[10000/AmpMax]
% Joint axis stroke limit [deg]
ri{2}.rj_margin = ones(1,6)*2;
ri{2}.rj_pos_limit_min = [-170, -100, -72, -190, -125, -360] + ri{2}.rj_margin; %[deg]     
ri{2}.rj_pos_limit_max = [170,  145,  240, 190, 125, 360] - ri{2}.rj_margin; %[deg]
% Joint Velocity Limit [deg/s]
%ri{2}.velboundary_max = [35, 35, 50, 50, 50, 50];    % [deg/s]
ri{2}.velboundary_max = 1/4*[370,310, 410, 550, 545, 1000];   % [deg/s]

% Boundary Collision 
ri{2}.boundary_min = [-1.0, -1.0, -0.30]';     % x  y  z
ri{2}.boundary_max = [1.0,  1.0,  100]';        % x  y  z
ri{2}.tcp_boundary_min = ri{2}.iniTcpPos_W(1:3)/1000 + [-0.25, -0.3, -0.18-0.02];     % x  y  z
ri{2}.tcp_boundary_max = ri{2}.iniTcpPos_W(1:3)/1000 + [0.4,  0.8+0.2,  0.4];        % x  y  z
% Self Collision 
ri{2}.RADIUS = [0.13, 0.12, 0.06, 0.06, 0.05];
ri{2}.BASE_Z = -ri{2}.J1Origin(3);
% Force Limit
ri{2}.Force_limit = [50; 50; 50; 10; 10; 10];   % Force:N   Torque:Nm

% Used in Dual arm coordination
si.DH = ri{1}.DH;
si.RADIUS = ri{1}.RADIUS;
si.BASE_Z = ri{1}.BASE_Z;    
si.OFFSET  = [0; -1.27; 0];
si.dstart = 0.150;   % 15cm
si.d0 = 0.050;       %  5cm

%% Robtic Toolbox model
si.rtb{1} = buildRTBmdl(ri{1});
si.rtb{2} = buildRTBmdl(ri{2});

%% Max Speed and Acceleration Time Constant of Internal Motion 
% Robot 1
% Internal Programed motion
    % Acceleration Time Constant
    ri{1}.AccelTime1_s = [320 320 320 320 320 320]*2;   % [msec]
    ri{1}.AccelTime2_s = ri{1}.AccelTime1_s*0.5;           % [msec]
    % Max speed of joint axis 
    %ri{1}.jvelmax = [20 20 20 20 20 20];                % [deg/s]   
    ri{1}.jvelmax = 2*[20 20 20 20 20 20];                % [deg/s]  
% Jog
    % Acceleration Time Constant
    ri{1}.jog_AccelTime1_s = [96 96 96 96 96 96]*4;     % [msec]
    ri{1}.jog_AccelTime2_s = ri{1}.jog_AccelTime1_s*0.5;   % [msec]
    % Max speed of joint axis 
    %ri{1}.jog_jvelmax = [20 20 20 20 20 20];            % [deg/s]
    ri{1}.jog_jvelmax = 2*[20 20 20 20 20 20];            % [deg/s]
% Hold stop
    % Acceleration Time Constant
    ri{1}.ext_hold_ta1 = 100;                           % [itp]  
    ri{1}.ext_hold_ta2 = 50;                            % [itp]

% Robot 2
% Internal Programed motion
    % Acceleration Time Constant
    ri{2}.AccelTime1_s = [320 320 320 320 320 320]*2;   % [msec]
    ri{2}.AccelTime2_s = ri{2}.AccelTime1_s*0.5;           % [msec]
    % Max speed of joint axis 
    %ri{2}.jvelmax = [20 20 20 20 20 20];                % [deg/s]   
    ri{2}.jvelmax = 2*[20 20 20 20 20 20];                % [deg/s]   
% Jog
    % Acceleration Time Constant
    ri{2}.jog_AccelTime1_s = [96 96 96 96 96 96]*4;     % [msec]
    ri{2}.jog_AccelTime2_s = ri{2}.jog_AccelTime1_s*0.5;   % [msec]
    % Max speed of joint axis 
    %ri{2}.jog_jvelmax = [20 20 20 20 20 20];            % [deg/s]
    ri{2}.jog_jvelmax = 2*[20 20 20 20 20 20];            % [deg/s]
% Hold stop
    % Acceleration Time Constant
    ri{2}.ext_hold_ta1 = 100;                           % [itp]  
    ri{2}.ext_hold_ta2 = 50;                            % [itp]
       
%% Mechanical Parameters
% Robot 1
% Motor torque constant
ri{1}.TrqCons = [0.326 0.326 0.155 0.0818 0.0829 0.0829]; %[Nm/Ap]
ri{1}.DSA2Ap = [20,20,20,20,10,10]/10000;   % [Ap/DSA]
% Gear reduction ratio
ri{1}.Rratio = [4240/37 121/1 2960/29 3360/46 2000/24 1200/29];
% Plus direction of robot joint axis against motor plus direction.
% 1:same direction,   -1:opposite direction
ri{1}.Motor2TP_sign = [1 -1 1 -1 1 -1];   
% Interaction Matrix
% robot motor pos [deg] --> robot joint axis [deg]
ri{1}.interaction_J56 = 12500/3;   % J6=J5motor/r.pa.interaction_J56+J6motor/r.pa.Rratio(6)*r.pa.sign.Motor2TP_sign(6)
ri{1}.interaction = diag(1./ri{1}.Rratio.*ri{1}.Motor2TP_sign);
ri{1}.interaction(6,5) = 1/ri{1}.interaction_J56;
% Inverse of Interaction Matrix
% robot joint axis[deg] --> Motor pos[2^20/rev]
ri{1}.k_rbj2mpls = inv(ri{1}.interaction)/360*2^20;
% Interaction Matrix2
% Motor pos[2^20/rev] -->robot joint axis[deg]
ri{1}.k_mpls2rbj = inv(ri{1}.k_rbj2mpls);
% Reduction vector
% Motor pos[2^20/rev] -->robot joint axis[deg]
% ri{1}.k_vec_mpls2rbj = diag(ri{1}.k_mpls2rbj)';

% Robot 2
% Motor torque constant
ri{2}.TrqCons = [0.326 0.326 0.155 0.0818 0.0829 0.0829]; %[Nm/Ap]
ri{2}.DSA2Ap = [20,20,20,20,10,10]/10000;   % [Ap/DSA]
% Gear reduction ratio
ri{2}.Rratio = [4240/37 121/1 2960/29 3360/46 2000/24 1200/29];
% Plus direction of robot joint axis against motor plus direction.
% 1:same direction,   -1:opposite direction
ri{2}.Motor2TP_sign = [1 -1 1 -1 1 -1];  
% Interaction Matrix
% robot motor pos [deg] --> robot joint axis [deg]
ri{2}.interaction_J56 = 12500/3;   % J6=J5motor/r.pa.interaction_J56+J6motor/r.pa.Rratio(6)*r.pa.sign.Motor2TP_sign(6)
ri{2}.interaction = diag(1./ri{2}.Rratio.*ri{2}.Motor2TP_sign);
ri{2}.interaction(6,5) = 1/ri{2}.interaction_J56;
% Inverse of Interaction Matrix
% robot joint axis[deg] --> Motor pos[2^20/rev]
ri{2}.k_rbj2mpls = inv(ri{2}.interaction)/360*2^20;
% Interaction Matrix2
%Motor pos[2^20/rev] -->robot joint axis[deg]
ri{2}.k_mpls2rbj = inv(ri{2}.k_rbj2mpls);
% Reduction vector
% Motor pos[2^20/rev] -->robot joint axis[deg]
% ri{2}.k_vec_mpls2rbj = diag(ri{2}.k_mpls2rbj)';

%% Initialization of Robot External Motion Command Variables 
% these data will be updated in init_tunable_params.m
si.extcmd.vcmd_t = zeros(si.ext_vel_max_cnt,si.AxisNum);
si.extcmd.vcmd_EndCnt_t = 1;
si.extcmd.inipos_t = si.iniJntPos;
si.extcmd.grippercmd = zeros(si.ext_vel_max_cnt,6);

%% 2nd Order Low Pass Filter (tsv time)
si.Filter.w0 = 2*pi*10;    % 10 Hz
si.Filter.D = 1;           % damping ratio
si.Filter.FilterC = tf(si.Filter.w0^2,[1,2*si.Filter.D*si.Filter.w0,si.Filter.w0^2]);
si.Filter.FilterD = c2d(si.Filter.FilterC,si.tsv);
si.Filter.num = cell2mat(si.Filter.FilterD.num);
si.Filter.den = cell2mat(si.Filter.FilterD.den);

% Lead Through Mode Filter (11/18/2015)
si.Filter2.w0 = 2*pi*2;    % 2 Hz
si.Filter2.D = 1;           % damping ratio
si.Filter2.FilterC = tf(si.Filter2.w0^2,[1,2*si.Filter2.D*si.Filter2.w0,si.Filter2.w0^2]);
si.Filter2.FilterD = c2d(si.Filter2.FilterC,si.tsv);
si.Filter2.num = cell2mat(si.Filter2.FilterD.num);
si.Filter2.den = cell2mat(si.Filter2.FilterD.den);

%% ATI Force Sensor
% % ATI mini 45 Force F/T Transducer (Berkeley FT16208)
ri{1}.CalibrationMatrix = ...
    [0.84694,-0.00013,2.25771,-47.38105,-2.19622,46.19011;
    -2.49558,54.11911,2.28525,-27.38248,2.24670,-26.76265;
    67.53655,4.50683,67.62002,1.77566,66.15999,3.66868;
    -0.01158,0.37397,-1.06950,-0.21997,1.08646,-0.12505;
    1.23392,0.08344,-0.66049,0.30482,-0.62385,-0.35227;
    0.00477,-0.68678,0.06877,-0.70011,0.03601,-0.69183];

% % ATI mini 45 Force F/T Transducer (Berkeley FT16527)
ri{2}.CalibrationMatrix = ...
    [0.21198,   0.42177,   5.08108, -47.93927,  -2.98636,  46.82127;
    -5.64667,  54.69388,   2.38259, -27.70532,   1.51383, -27.21215;
	67.88652,   3.10312,  64.94450,   3.80996,  70.00385,   4.15619;
	-0.03062,   0.37869,  -1.06126,  -0.25899,   1.09080,  -0.12045;
	 1.25548,   0.05501,  -0.66600,   0.29205,  -0.61155,  -0.36301;
	 0.09090,  -0.69858,   0.07311,  -0.70422,   0.02548,  -0.69853];

si.FS_ID_On = 0;    % 1- do force sensor calibration; 0- not do calibration

% Parameters for LTT and Repulsive force
si.LTT_Gain_High = [0.025, 0.025, 0.025, 0.8, 0.5, 0.8];
si.LTT_Gain_Low = si.LTT_Gain_High / 3;   
si.Frep_Gain = [0.3, 0.3, 0.3];
si.LTT_DeadZone = [1.5, 1.5, 1.5, 0.1, 0.1, 0.1];                           % Dead zone for LTT force
si.LTT_Speed_Limit = [0.15, 0.15, 0.15, 20/180*pi, 20/180*pi, 20/180*pi];   % Speed limit for LTT (m/sec, rad/sec), to cut off excessive force to avoid unstable motions during sudden turn overs

% Force calibrate for gripper 
if si.FS_ID_On
    ri{1}.ATI_Offset = zeros(1,6);   
    ri{1}.PayloadG =  zeros(1,3);                                               % Gravity force of payload in robot base frame      
    ri{1}.PayloadCG = zeros(1,3);                                               % CG of Payload in robot sensor frame                     
else
    % without Handle Bar
    ri{1}.ATI_Offset = [-17.2517, -14.7141, -12.7484, -0.31088, 0.55276, 0.097411]; 
    ri{1}.PayloadG = [0, 0, -13.71];                                        % Gravity force of payload in robot base frame
    ri{1}.PayloadCG = [-0.0082766,  -0.0029982,     0.06217];                         % CG of Payload in robot sensor frame 
end
ri{1}.L_ATI2TCP = [0, 0, 0.200];                                                    % Distance from ATI sensor to TCP in robot tool frame
%ri{1}.L_ATI2LTT = [0.035/2+0.0195+0.1916-0.0576, 0.010, 0.027+0.046-0.005];         % Distance from ATI sensor to LTT in robot tool frame
% remove Handle Bar
ri{1}.L_ATI2LTT = [0, 0, 0.027+0.046-0.005]; 

% Force calibrate for gripper 
if si.FS_ID_On
    ri{2}.ATI_Offset = zeros(1,6);   
    ri{2}.PayloadG =  zeros(1,3);                                               % Gravity force of payload in robot base frame      
    ri{2}.PayloadCG = zeros(1,3);                                               % CG of Payload in robot sensor frame                    
else
    % without Handle Bar
    ri{2}.ATI_Offset = [-4.0331, -8.01986, -22.466, -0.21798, 0.43101, 0.1096];
    ri{2}.PayloadG =  [0, 0, -13.2949];                                         % Gravity force of payload in robot base frame
    ri{2}.PayloadCG = [-0.011225, -0.0054653, 0.047827];                         % CG of Payload in robot sensor frame 
end
ri{2}.L_ATI2TCP = [0, 0, 0.200];                                                    % Distance from ATI sensor to TCP in robot tool frame
%ri{2}.L_ATI2LTT = [0.035/2+0.0195+0.1916-0.0576, -0.010, 0.027+0.046-0.005];        % Distance from ATI sensor to LTT in robot tool frame
% remove Handle Bar
ri{2}.L_ATI2LTT = [0, 0, 0.027+0.046-0.005]; 

%% Peg-hole insertion
si.peghole = load('GMRpar.mat');  % peg-hole insertion GMR parameters

%% 
si.ri = ri;
fprintf('\n---> Parameters for LRMate200 robot system have been set.\n\n')
