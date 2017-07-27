%% comunication with Ubuntu
% Run the following commands after setting up your PC's ros master IP, and connecting it to kinect
setenv('ROS_MASTER_URI','http://192.168.1.40:11311')
setenv('ROS_IP','192.168.1.90')
rosinit()

%% Brake Off
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(si.ParamSgnID, 'off');

%% Subsribe to ROS topic
sub = rossubscriber('tracker/object'); % your PC, as ros master, should be publishing this topic (tracked obj) now
received_data = receive(sub,3); % wait for at most 3 seconds.
points = received_data.Rope.Nodes; % N*1 vector, each element: .X, .Y, .Z
N = size(points,1); % total num of nodes on a rope

%% Finally run the robot
rigidCompensate = 0;
[Z_Data_Test2, warp] = LTT_Warping_CPD(LTT_Data_Train, points_W(:,1:2), points_Test_W(:,1:2), si , '2D', WarpIndex, rigidCompensate);

Traj_Download_Run(Z_Data_Test2, si, 'Download', 'Run');
