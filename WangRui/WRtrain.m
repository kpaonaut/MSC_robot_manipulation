% record the bullet position
clear points_U
sub=rossubscriber('tracker/object');
received_data = receive(sub,3); 
points = received_data.Rope.Nodes;
%load('F:\WANGRUI\V4.0\data\Rope\Straightening.mat') ;%·only use during the test
points_U = [[points.X]', [points.Y]', [points.Z]']; % convert points(i).X,Y,Z to points_T_U(i, 1,2,3)
% !!! Naming convention:
% 'Rope_idx1_num2', where
% idx1 indicates the index of the robot, namely, this pic in training is taken
% for which robot to warp in testing;
% num2 indicates the index of the step, namely, the index for this pic in
% this robot's all training pics.
save('F:\WANGRUI\V4.0\data\Rope\Rope_Goal.mat', 'points_U');
% !!!!! IN typical TSM-RPM, should store 2 pics for robot 1 and 2 pics for robot 2!
%% record the teaching trajectory
LTT_Data_Train = Load_LTT(si);
LTT_Data_Train = LTT_Data_Refine(LTT_Data_Train, si);
save('F:\WANGRUI\V4.0\data\Rope\Traj_Train.mat','LTT_Data_Train');
% save('F:\TeTang\V4.0\data\Rope\received_data.mat','received_data')
%% show the teaching trajectory
    disp('======================================================================');
    fig1_handle = figure(2);
    set(fig1_handle,'position', [962 42 958 434]);
    % save as LTT_Data_UCBtest for CFS check
    LTT_Data_UCBtest =LTT_Data_Train ;
    save('F:\TeTang\V4.0\CFS_ver1.1\dataLTT\UCBTest.mat', 'LTT_Data_UCBtest');
    % CFS check (graphics display of robots carrying out traj -WR)
    CFS_Main