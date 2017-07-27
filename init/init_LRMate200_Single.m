%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Main File
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 06/18/2015       (Based on H.Nakagawa's Dual Robot Experimentor)
%  MSC Lab, U.C.Berkeley
%  Modified on 11/11/2015 by Te Tang
%  Modified on 05/20/2016 by Wenjie Chen for FANUC setups
%  Modified on 04/12/2017 by Te Tang for Berkeley setup
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

function si = init_LRMate200_Single()

fprintf('Now starting FANUC Single Arm Robot Experimentor ...\n\n');

%% Initialization
si.robot_num = 1;
ri = cell(si.robot_num,1);

init_bus_objects;           % Initilize customized bus objects
% init_model_workspace;     % Not necessary now, will be auto executed by referenced model.
check_robot_model_copy;
unload_all_models;

%% Robot Number
ri{1}.robot_no = 1;

%% Hardware On/Off State
si.wasStopped = 1;          % default: stopped 
ri{1}.wasBrakeOff = 0;      % default: brake on

%% Robot Master Count Parameter 
% Set the robot master count parameter in R-30iB robot control system.
% The robot master count represents the encorder value at the zero posture of the robot. 
% Each Single Robot has its own MasterCount !!!!!!!!!!!!!!!!!!

% % UCB LRMate200iD/7L No.1
ri{1}.MasterCount = [-1160254 8259621 1149690 -184284 3990732 -85885]; % MasterCount updated in 10/16/2016
% 
% % UCB LRMate200iD/7L No.2
% ri{2}.MasterCount = [105567 -350832 281032 -330594 551715 -486217];  % MasterCount updated in 10/16/2016

% % FANUC Internship  LRMate200iD/7L No.1 (D53965,R15201968,E15132888)
% ri{1}.MasterCount=[333096 -6773653 8926657 129130 -80980 -6290601];
% % FANUC Internship  LR Mate 200iD/7L No.2 (D53966,R15201969,E15132889)
% ri{2}.MasterCount=[82449 -5809569 9136228 62648 -239540 -12282073];     

% FANUC Internship  LRMate200iD No.1 (D49151,R14601767,E14630099)
% ri{1}.MasterCount=[-1858 80204 5649 -10777 -113359 -149940];

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
si.WOrigin = [0, 0, 0];      % define world frame origin as the origin of the design space
si.T_DS2W = [rpy2r(si.RPY_W2DS, 'zyx') * [eye(3) -si.WOrigin']; 0 0 0 1];   % transformation from the design space frame to world frame

% Camera calibration center coordinates (this is for Ensenso camera)
si.camera_type = 'ensenso';     % ensenso camera, currently kinect sensor is not calibrated
si.k2 = [];                     % set k2 (kinect v2) object to empty
si.CalibGridCenter_W = [0, 0, 0];   % Ensenso is not calibrated for single arm system

% Robot coordinates systems
ri{1}.BaseCenter = [0, 0, 0];                               % center position of robot base
ri{1}.J1Origin = ri{1}.BaseCenter + [0, 0, 0.330];          % robot J1 origin in design space frame
ri{1}.RPY_DS2B = [0, 0, 0];                                 % rotation angle RPY from design space frame to robot base (J1) frame
ri{1}.T_B2DS = [[rpy2r(ri{1}.RPY_DS2B, 'zyx') ri{1}.J1Origin']; 0 0 0 1];   % transformation from robot base (J1) frame to design space
ri{1}.T_B2W = si.T_DS2W * ri{1}.T_B2DS;                     % transformation from robot base (J1) frame to world frame

%% DH Parameters
% DH parameter for LR Mate 200iD/7L
ri{1}.n = si.AxisNum;       % robot axis number
ri{1}.DH = ...   %  J1          J2          J3          J4          J5          J6 
             [      0           -pi/2       0           0           0           0;          % Theta0 (rad)
                    0           0           0           -0.420      0           -0.08;      % D (m)
                    0.050       0.440       0.035       0           0           0;          % A (m)
                    -pi/2       pi          -pi/2       pi/2        -pi/2       pi];        % alpha (rad)    
ri{1}.J62Gripper = 0.027+0.0157+0.027;  % Adapter + F/T sensor + Adapter
ri{1}.Gripper2TCP = 0.200-0.027;        % Gripper with Adapter - Adapter
ri{1}.DH(:,end+1) = [0; ri{1}.J62Gripper + ri{1}.Gripper2TCP; 0; 0];    % DH for robot tool (gripper)
ri{1}.DH(:,end+1) = [0;0.1143;0.206;0];     % Workpiece DH ------------------------------------------------------------- To be changed

% DH parameter for LR Mate 200iD
% ri{1}.n = si.AxisNum;       % robot axis number
% ri{1}.DH = ...   %  J1          J2          J3          J4          J5          J6 
%              [      0           -pi/2        0           0           0           0;          % Theta0 (rad)
%                     0           0           0           -0.335      0           -0.08;      % D (m)
%                     0.050       0.330       0.035       0           0           0;          % A (m)
%                     -pi/2        pi           -pi/2       pi/2        -pi/2       pi];        % alpha (rad)    
% ri{1}.J62Gripper = 0.027+0.0157+0.027;  % Adapter + F/T sensor + Adapter
% ri{1}.Gripper2TCP = 0.238-0.027;        % Gripper with Adapter - Adapter
% ri{1}.DH(:,end+1) = [0; ri{1}.J62Gripper + ri{1}.Gripper2TCP; 0; 0];    % DH for robot tool (gripper)
% ri{1}.DH(:,end+1) = [0;0.1143;0.206;0];     % Workpiece DH ------------------------------------------------------------- To be changed

%% Initial Position Setting
% This is to set robot dependent initial position. This will be used to
% reset initial position values using parameter tuning after model building
ri{1}.iniJ1Off = 0;    % initial J1 offset for robot 1 (deg)
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
% FF controller model (0-1: LRMate200iD7L with/without gripper; 2-3: LRMate200iD with/without gripper)
ri{1}.model_no = 0;    

%% Safety Check Parameter
% Safety Parameter for Robot 1
% Motor torque limit.  10000 represents amp max torque.
ri{1}.SafetyTrqLimit = [3000 3000 3000 2000 2000 2000]; %[10000/AmpMax]
% Joint axis stroke limit [deg]
ri{1}.rj_margin = ones(1,6)*2;
ri{1}.rj_pos_limit_min = [-170, -100, -72, -190, -125, -360] + ri{1}.rj_margin; %[deg]     
ri{1}.rj_pos_limit_max = [170,  145,  240, 190, 125, 360] - ri{1}.rj_margin; %[deg]
% Joint Velocity Limit [deg/s]
ri{1}.velboundary_max = [35, 35, 50, 50, 50, 50];    % [deg/s]
% Boundary Collision 
ri{1}.boundary_min = [-1.0, -1.0, -0.33]';     % x  y  z ;for FANUC LRMate 200iD [-1.0, -1.0, -0.3]'
ri{1}.boundary_max = [1.0,  1.0,  100]';        % x  y  z
ri{1}.tcp_boundary_min = ri{1}.iniTcpPos_W(1:3)/1000 + [-0.25, -0.8, -0.4];     % x  y  z
ri{1}.tcp_boundary_max = ri{1}.iniTcpPos_W(1:3)/1000 + [0.4,  0.3,  0.4];  % x  y  z; for FANUC LRMate 200iD [0.5,  0.8,  0.8]
% Self Collision 
ri{1}.RADIUS = [0.13, 0.12, 0.06, 0.06, 0.05];
ri{1}.BASE_Z = -ri{1}.J1Origin(3);    
% Force Limit
ri{1}.Force_limit = [50; 50; 50; 10; 10; 10];   % Force:N   Torque:Nm

%% Robtic Toolbox model
si.rtb{1} = buildRTBmdl(ri{1});

%% Max Speed and Acceleration Time Constant of Internal Motion 
% Robot 1
% Internal Programed motion
    % Acceleration Time Constant
    ri{1}.AccelTime1_s = [320 320 320 320 320 320]*2;   % [msec]
    ri{1}.AccelTime2_s = ri{1}.AccelTime1_s*0.5;           % [msec]
    % Max speed of joint axis 
    ri{1}.jvelmax = [20 20 20 20 20 20];                % [deg/s]   
% Jog
    % Acceleration Time Constant
    ri{1}.jog_AccelTime1_s = [96 96 96 96 96 96]*4;     % [msec]
    ri{1}.jog_AccelTime2_s = ri{1}.jog_AccelTime1_s*0.5;   % [msec]
    % Max speed of joint axis 
    ri{1}.jog_jvelmax = [20 20 20 20 20 20];            % [deg/s]
% Hold stop
    % Acceleration Time Constant
    ri{1}.ext_hold_ta1 = 100;                           % [itp]  
    ri{1}.ext_hold_ta2 = 50;                            % [itp]

%% Mechanical Parameters
% Robot 1
% Motor torque constant
ri{1}.TrqCons = [0.326 0.326 0.155 0.0818 0.0829 0.0829]; %[Nm/Ap]
ri{1}.DSA2Ap = [20,20,20,20,10,10]/10000;   % [Ap/DSA]
% Gear reduction ratio
ri{1}.Rratio = [4240/37 121/1 2960/29 3360/46 2000/24 1200/29];% LR Mate 200iD/7L
%ri{1}.Rratio = [3570/39 101/1 80/1 3360/46 2000/24 1200/29]; % LR Mate 200iD

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
% ATI mini 45 Force F/T Transducer (Berkeley FT16208)
ri{1}.CalibrationMatrix = ...
    [0.84694,-0.00013,2.25771,-47.38105,-2.19622,46.19011;
    -2.49558,54.11911,2.28525,-27.38248,2.24670,-26.76265;
    67.53655,4.50683,67.62002,1.77566,66.15999,3.66868;
    -0.01158,0.37397,-1.06950,-0.21997,1.08646,-0.12505;
    1.23392,0.08344,-0.66049,0.30482,-0.62385,-0.35227;
    0.00477,-0.68678,0.06877,-0.70011,0.03601,-0.69183];
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

si.FS_ID_On = 0;    % 1- do force sensor calibration; 0- not do calibration

% Parameters for LTT and Repulsive force
si.LTT_Gain_High = [0.025, 0.025, 0.025, 0.8, 0.5, 0.8];
si.LTT_Gain_Low = si.LTT_Gain_High / 3;   
si.Frep_Gain = [0.3, 0.3, 0.3];
si.LTT_DeadZone = [1.5, 1.5, 1.5, 0.1, 0.1, 0.1];                           % Dead zone for LTT force
si.LTT_Speed_Limit = [0.15, 0.15, 0.15, 20/180*pi, 20/180*pi, 20/180*pi];   % Speed limit for LTT (m/sec, rad/sec), to cut off excessive force to avoid unstable motions during sudden turn overs

% ATI mini45 FT 16919 
% ri{1}.CalibrationMatrix = ...
%     [  1.61542   0.13940  -0.25184 -47.81100  -1.97928  47.38927;
%       -3.99033  54.45251   0.40798 -27.35117   2.32758 -27.79827;
%       67.56209   2.44017  67.68324   1.93287  66.07174   2.13007;
%       -0.03640   0.37190  -1.07449  -0.23151   1.06973  -0.14144;
%        1.24836   0.06032  -0.63264   0.30189  -0.60630  -0.34957;
%        0.02589  -0.69871   0.05584  -0.70505   0.00796  -0.70081];

% Force calibrate for gripper 
if si.FS_ID_On
    ri{1}.ATI_Offset = zeros(1,6);   
    ri{1}.PayloadG =  zeros(1,3);                                               % Gravity force of payload in robot base frame      
    ri{1}.PayloadCG = zeros(1,3);                                               % CG of Payload in robot sensor frame                     
else
    ri{1}.ATI_Offset = [-17.8034, -14.8311, -12.9552, -0.32722, 0.6174, 0.17942];   
    ri{1}.PayloadG = [0, 0, -12.4403];                                         % Gravity force of payload in robot base frame
    ri{1}.PayloadCG = [0.0051545, -0.0027871, 0.071898];                          % CG of Payload in robot sensor frame 
end
ri{1}.L_ATI2TCP = [0, 0, 0.200];                                                    % Distance from ATI sensor to TCP in robot tool frame
%ri{1}.L_ATI2LTT = [0.051/2+0.0195+0.1916-0.0576, 0.020, 0.027+0.069-0.002];         % Distance from ATI sensor to LTT in robot tool frame
% remove Handle Bar
ri{1}.L_ATI2LTT = [0, 0, 0.027+0.046-0.005]; 
%% Peg-hole insertion
si.peghole = load('GMRpar.mat');  % peg-hole insertion GMR parameters

%% 
si.ri = ri;
fprintf('\n---> Parameters for LRMate200 robot system have been set.\n\n')
