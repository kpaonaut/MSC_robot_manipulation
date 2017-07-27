% record the bullet position
clear points_U
sub=rossubscriber('tracker/object');
received_data = receive(sub,3); 
points = received_data.Rope.Nodes;
%load('F:\TeTang\V4.0\data\Rope\Bullet_Points_RopeKnot_changhao1.mat') ;%·only use during the test
N=size(points,1);
for i=1:N
        points_U(i,1)=points(i).X;
        points_U(i,2)=points(i).Y;
        points_U(i,3)=points(i).Z;
end
save('F:\TeTang\V4.0\data\Rope\Bullet_Train_RopeKnot_changhao_unknot_1.mat', 'points_U');

%% record the teaching trajectory
LTT_Data_Train = Load_LTT(si);
LTT_Data_Train = LTT_Data_Refine(LTT_Data_Train, si);
save('F:\TeTang\V4.0\data\Rope\Traj_Train_RopeKnot_changhao_unknot_1.mat','LTT_Data_Train');
% save('F:\TeTang\V4.0\data\Rope\received_data.mat','received_data')
%% show the teaching trajectory
    disp('======================================================================');
    fig1_handle = figure(2);
    set(fig1_handle,'position', [962 42 958 434]);
    % save as LTT_Data_UCBtest for CFS check
    LTT_Data_UCBtest =LTT_Data_Train ;
    save('F:\TeTang\V4.0\CFS_ver1.1\dataLTT\UCBTest.mat', 'LTT_Data_UCBtest');
    % CFS check
    CFS_Main