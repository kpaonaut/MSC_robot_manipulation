%% Log data from TargetPC to HostPC
clc;
% Achieve TCP_T, JointPos, TiltAngle signal index

%%%%%%%%% Use tg.ShowSignals = 'on'   to see all the signals' corresponding path information 

TCP_T_index = getsignalid(tg,'Measurement/TiltEstimate/p1/s1');
JointPos_index = getsignalid(tg,'Measurement/TiltEstimate/p1/s17');
TiltAngle_index = getsignalid(tg,'Measurement/TiltEstimate/p1/s23');

TCP_T_row = zeros(1,16);
for i = 1:16
    TCP_T_row(i) = getsignal(tg,TCP_T_index+i-1);
end
TCP_T = reshape(TCP_T_row,4,4);

JointPos_deg = zeros(6,1);
for i = 1:6
    JointPos_deg(i) = getsignal(tg,JointPos_index+i-1);
end

TiltAngle = getsignal(tg,TiltAngle_index);

%% Calculate the new TCP Transformation Matrix
r = 25.369/2/1000;    % peg radius (half diameter) 
deltaX = r*(1-cosd(TiltAngle));
deltaZ = r*(1-cosd(TiltAngle))/(1+cosd(TiltAngle))*sind(TiltAngle);
deltaY = 0;

PosOffest =  [ cosd(-TiltAngle),    0,  sind(-TiltAngle); ...  
                0               ,    1,           0;       ...
               -sind(-TiltAngle),    0,  cosd(-TiltAngle)]*[-deltaX ; deltaY ; deltaZ];    

T_Hole2TCP = [ cosd(-TiltAngle),    0,  sind(-TiltAngle),   PosOffest(1);
               0               ,    1,           0      ,   PosOffest(2);
              -sind(-TiltAngle),    0,  cosd(-TiltAngle),   PosOffest(3);
               0,                   0,           0,              1       ];

TCP_T_new = TCP_T*T_Hole2TCP;

% at FANUC
% TCP_T_new = TCP_T_new*[1 0 0 0.0047;    ...        % 0.47cm x+  
%                        0 1 0 -0.0015;    ...       % 0.15 cm y-
%                        0 0 1 -0.007;...             % 0.7 cm above the hole
%                        0 0 0 1];  

% at Berkeley                   
TCP_T_new = TCP_T_new*[1 0 0 0.0085;    ...        % 0.47cm x+  
                       0 1 0 -0.0021;    ...       % 0.15 cm y-
                       0 0 1 -0.01;...             % 0.7 cm above the hole
                       0 0 0 1];                    

%% Inverse Kinematics by RTB
TiltAngle
JointPosNew_deg = 180/pi*rtb{1}.ikcon(TCP_T_new, JointPos_deg/180*pi)'

%% Generate Compensate Trajectory
q1 = JointPos_deg'/180*pi;
q2 = (JointPos_deg' + [0 -2 0 0 0 0])/180*pi;
q3 = (JointPosNew_deg'+ [0 0 0 0 0 0] )/180*pi;
q4 = JointPosNew_deg'/180*pi;

qr = [q1;q2;q3;q4];
TimeVec = [1,2,1]*1000;   %unit: ms

CSVwirtePath = '..\Data\ext_vcmd_csv\TiltCompensate.csv';
addpath(genpath('..\Data'));

% Generate external command and download to target PC
extcmd = ExtCmdGenerate(qr, TimeVec, si, ri{1});
ExtCmdDownload(si.ParamSgnID{1}, extcmd);
