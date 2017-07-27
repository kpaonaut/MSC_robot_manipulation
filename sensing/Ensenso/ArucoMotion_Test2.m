%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       *Test file* for Aruco detection and motion generation
%       Please run cell by cell, instead of running in once
%       Here generate T_w without the use of Ensenso depth xyz data  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Move Robot 1 to [0;0;0.40] 
output = FrameTransform([0;0;0.40], 'points', 'W2B', si, 1)
TCP_xyz = output'*1000     % mm

opt.robot_idx = 1;   % robot index set for which operations should be conducted (default: all robots)
opt.tcp = 1;         % 0 - joint position (default); 1 - tcp position
opt.SI_jnt = 0;      % 0 - degree (default); 1 - radian
opt.SI_tcp = 0;      % 0 - degree, mm (default); 1 - radian, m
DesTCP_xyzwpr = [TCP_xyz,180,0,129]
tp_pos_run(si, DesTCP_xyzwpr, opt)

%% Open Cam
nxOpenCam

%% Detect Marker 5 times by detectAruco. Get Marker center and Rodrigues angle
for i = 1:5
    [image{i},ids{i},rvecs{i},tvecs{i}] = detectAruco;
    imshow(image{i})
    pause()
end

%% Close Cam
nxCloseCam

%% Average 5 times of measurement
for j = 1:5
    temp_t = tvecs{j};
    temp_r = rvecs{j};
    t2(:,j) = reshape(temp_t(1,1,:),3,1);
    t1(:,j) = reshape(temp_t(2,1,:),3,1);
    r2(:,j) = reshape(temp_r(1,1,:),3,1);
    r1(:,j) = reshape(temp_r(2,1,:),3,1);
end

t1_avg = sum(t1,2)/5;
r1_avg = sum(r1,2)/5;
t2_avg = sum(t2,2)/5;
r2_avg = sum(r2,2)/5;

%% Get Transformation matrix 

R1 = cv.Rodrigues(r1_avg);
R2 = cv.Rodrigues(r2_avg);
T1 = eye(4);
T1(1:3,1:3) = R1; T1(1:3,4) = t1_avg;
T2 = eye(4);
T2(1:3,1:3) = R2; T2(1:3,4) = t2_avg;

%% Calculate Transformation from Camera to World frame

% +		cv::Matx<double,3,1>	{val=0x000000a95d77f068 {-2.1952946465895820, 2.1828689748560954, 0.061104656182000498} }	cv::Matx<double,3,1>
% +		cv::Matx<double,3,1>	{val=0x000000a95d77f098 {0.11972139127241620, -0.074586002195200438, 1.4275547883342286} }	cv::Matx<double,3,1>
% +		cv::Matx<double,3,1>	{val=0x000000a95d77f068 {-2.1934545643508123, 2.1810417533448003, 0.067564148058786633} }	cv::Matx<double,3,1>
% +		cv::Matx<double,3,1>	{val=0x000000a95d77f098 {0.11974646388207899, -0.074738731222519894, 1.4287517098906621} }	cv::Matx<double,3,1>
% +		cv::Matx<double,3,1>	{val=0x000000a95d77f068 {-2.1942407362246907, 2.1817924485077325, 0.066006799283592807} }	cv::Matx<double,3,1>
% +		cv::Matx<double,3,1>	{val=0x000000a95d77f098 {0.11969585253346522, -0.074634877578992384, 1.4281180286200705} }	cv::Matx<double,3,1>
% +		cv::Matx<double,3,1>	{val=0x000000a95d77f068 {-2.1924603568853454, 2.1804357270034167, 0.059241142185445662} }	cv::Matx<double,3,1>
% +		cv::Matx<double,3,1>	{val=0x000000a95d77f098 {0.11972615171652672, -0.074624193851229842, 1.4281096935507529} }	cv::Matx<double,3,1>
% +		cv::Matx<double,3,1>	{val=0x000000a95d77f068 {-2.1922952750648470, 2.1802848071908993, 0.065830135199070355} }	cv::Matx<double,3,1>
% +		cv::Matx<double,3,1>	{val=0x000000a95d77f098 {0.11975715664166564, -0.074751868561184917, 1.4289367943440183} }	cv::Matx<double,3,1>

r_board = [-2.1952946465895820, 2.1828689748560954, 0.061104656182000498;
           -2.1934545643508123, 2.1810417533448003, 0.067564148058786633;
           -2.1942407362246907, 2.1817924485077325, 0.066006799283592807;
           -2.1924603568853454, 2.1804357270034167, 0.059241142185445662;
           -2.1922952750648470, 2.1802848071908993, 0.065830135199070355]';
       
t_board = [0.11972139127241620, -0.074586002195200438, 1.4275547883342286;
           0.11974646388207899, -0.074738731222519894, 1.4287517098906621;
           0.11969585253346522, -0.074634877578992384, 1.4281180286200705;
           0.11972615171652672, -0.074624193851229842, 1.4281096935507529;
           0.11975715664166564, -0.074751868561184917, 1.4289367943440183]';       

r_avg = sum(r_board,2)/5;
t_avg = sum(t_board,2)/5;

R_A2C = cv.Rodrigues(r_avg);
T_A2C = eye(4);
T_A2C(1:3,1:3) = R_A2C;  T_A2C(1:3,4) = t_avg;

T_A2W = [[eye(3);zeros(1,3)],[0.0755;-0.1258;0.2825;1]];

T_C2W = T_A2W * inv(T_A2C)


%% Process Marker info to get T in World frame (only use Ensenso RGB data, not use depth xyz data here)
ArucoMarker = struct;
ArucoMarker.id = [];
ArucoMarker.rvecs = {};
ArucoMarker.tvecs = {};

for iteration = 1:5
    [image,ids,rvecs,tvecs] = detectAruco;
    disp(ids')
    for i = 1:numel(ids)
        if isempty(find(ArucoMarker.id == ids(i)))
            ArucoMarker.id(end+1) = ids(i);
            ArucoMarker.rvecs{end+1} = [];
            ArucoMarker.tvecs{end+1} = [];
        end
        index = find(ArucoMarker.id == ids(i));
        ArucoMarker.rvecs{index}(:,end+1) =  [rvecs(i,1,1);rvecs(i,1,2);rvecs(i,1,3)];
        ArucoMarker.tvecs{index}(:,end+1) =  [tvecs(i,1,1);tvecs(i,1,2);tvecs(i,1,3)];
    end
end

for i = 1:numel(ArucoMarker.id)
    ArucoMarker.rvecs_avg(:,i) =  mean( ArucoMarker.rvecs{i},2);
    ArucoMarker.tvecs_avg(:,i) =  mean( ArucoMarker.tvecs{i},2);
    ArucoMarker.T_2C_avg{i} =  [[cv.Rodrigues(ArucoMarker.rvecs_avg(:,i)),ArucoMarker.tvecs_avg(:,i)];[0,0,0,1]];
    ArucoMarker.T_2W_avg{i} = T_C2W * ArucoMarker.T_2C_avg{i};
end

%% Move Robot to 10cm above Marker 
output = FrameTransform(ArucoMarker.T_2W_avg{1}(1:3,4), 'points', 'W2B', si, 1)
TCP_xyz = output'*1000     % mm
TCP_xyz = TCP_xyz + [0,0,100]

opt.robot_idx = 1;   % robot index set for which operations should be conducted (default: all robots)
opt.tcp = 1;         % 0 - joint position (default); 1 - tcp position
opt.SI_jnt = 0;      % 0 - degree (default); 1 - radian
opt.SI_tcp = 0;      % 0 - degree, mm (default); 1 - radian, m

DesTCP_xyzwpr = [TCP_xyz,180,0,129]
tp_pos_run(si, DesTCP_xyzwpr, opt)