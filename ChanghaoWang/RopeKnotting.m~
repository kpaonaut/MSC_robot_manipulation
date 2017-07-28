%% comunication with Ubuntu
% setenv('ROS_MASTER_URI','http://192.168.1.40:11311')
% setenv('ROS_IP','192.168.1.90')
% rosinit()
sub=rossubscriber('tracker/object');
%% Brake Off
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(si.ParamSgnID, 'off');
%%  choose the state
received_data = receive(sub,3);
points = received_data.Rope.Nodes;
N=size(points,1);
for i=1:N
    points_Test_U(i,1)=points(i).X;
    points_Test_U(i,2)=points(i).Y;
    points_Test_U(i,3)=points(i).Z; % points
end
for x1 = 1:4 % find the closest training data amongst 4 sets of existing training data -WR
    load(['F:\TeTang\V4.0\data\Rope\Bullet_Train_RopeKnot_changhao_shorttrain_',num2str(x1),'.mat']) ;
    N1=size(points_U,1);
    minimum=min([N,N1]);
    points_U=points_U(1:minimum,:);
    points_Test_U=points_Test_U(1:minimum,:); % only include the least num of nodes in both test and train ropes. hard code?? -WR
    y = points_U-points_Test_U; % diff between train and test, [dx, dy, dz] for each row
    for x2 = 1:minimum     % calculate the relative distance
        z(x2,:)=y(x2,1:2)-y(1,1:2);
    end
    distance(x1)=norm(z,2);
end
x=find(distance==min(distance)); % the closest training data -WR
distance=sort(distance);
if distance(2)-distance(1)>0.2
    state=x;
    disp(['this state is similar to the ',num2str(state)',' step']);
else  % if Least Square Method doesn't works, then use CPD method
    disp('Least Square Method cannot tell, we will try CPD method.');
    for x1 = 1:4
        %     load(['F:\TeTang\V4.0\data\Rope\Bullet_Train_RopeKnot_changhao_',num2str(x1),'.mat']) ;
        load(['Bullet_Train_RopeKnot_changhao_shorttrain_',num2str(x1),'.mat']) ;
        opt.method='nonrigid'; % use nonrigid registration
        opt.viz=1;              % show every iteration
        opt.outliers=0.5;         % don't account for outliers
        opt.fgt=0;              % do not use FGT (default)
        opt.normalize=1;        % normalize to unit variance and zero mean before registering (default)
        opt.corresp=1;          % compute correspondence vector at the end of registration (not being estimated by default)
        opt.max_it=100;         % max number of iterations
        opt.tol=1e-9;          % tolerance
        [Transform, C, cost(x1)]=findstate_CPD(points_U,points_Test_U, opt);
    end
    state=find(cost==min(cost));
    disp(['this state is similar to the ',num2str(state),' step']);
end
clear y distance cost

%% Main Step % four steps in total for a rope-knotting process -WR
RopeStep=state;
while RopeStep<=4
    % U: ubuntu coord, the kinect coord; W: world frame; B: robot frame -WR
    T_W2U=transl((22.5*2.54+2.486*5)/100,-2.486*2.5/100,0)*trotz(-pi/2); %transformation matrix; transl(...): translation, parallel move matrix -WR
    T_B2U=inv(si.ri{1}.T_B2W)*T_W2U; % 
    if RopeStep~=state  %dismiss the first step % check if the prev step succeeded -WR
        %check whether prev step succeed
        clear y distance z cost

        received_data = receive(sub,3);
        points_New = received_data.Rope.Nodes;
        N_New=size(points_New,1);
        for i=1:N_New
            points_New_U(i,1)=points_New(i).X;
            points_New_U(i,2)=points_New(i).Y;
            points_New_U(i,3)=points_New(i).Z;
        end
        load(['Bullet_Train_RopeKnot_changhao_shorttrain_',num2str(RopeStep+1),'.mat']) ;
        [Transform, C, cost(1)]=findstate_CPD(points_New_U,points_Test_U, opt);
        [Transform, C, cost(2)]=findstate_CPD(points_New_U,points_U, opt);
        detection=find(cost==min(cost));
        disp(['this state is similar to the ',num2str(detection),' step']);
        if detection==RopeStep+1
            disp('Last Step succeed');
            RopeStep=RopeStep+1;
        else
            disp('robot did not grasp the rope');
        end
    end
    clear LTT_Data_Train LTT_Data_Test  Z_Data_Test2 points_U points_W points_Test_U points_Test_W
    load(['F:\TeTang\V4.0\data\Rope\Traj_Train_RopeKnot_changhao_shorttrain_',num2str(RopeStep),'.mat']) ;
    load(['F:\TeTang\V4.0\data\Rope\Bullet_Train_RopeKnot_changhao_shorttrain_',num2str(RopeStep),'.mat']) ;
    received_data = receive(sub,3);
    points = received_data.Rope.Nodes;
    N = size(points,1);
    N1= size(points_U,1);
    for i=1:N
        points_Test_U(i,1)=points(i).X;
        points_Test_U(i,2)=points(i).Y;
        points_Test_U(i,3)=points(i).Z;
    end
    points_U=[points_U,ones(N1,1)];
    points_W=(T_W2U*points_U.').';
    points_W=points_W(:,1:3);
    points_U=points_U(:,1:3); % all training points transformed to world finished -WR
    points_Test_U=[points_Test_U,ones(N,1)];
    points_Test_W=(T_W2U*points_Test_U.').';
    points_Test_W=points_Test_W(:,1:3);
    points_Test_U=points_Test_U(:,1:3); % testing points transformed to world finished -WR
    switch RopeStep
        case 1
            WarpIndex=1; k=3; %k represents the replay speed factor
        case 2
            WarpIndex=1;k=2;
        case 3
            WarpIndex=1;k=2;
        case 4
            WarpIndex=2;k=3;
    end
    LTT_Data_Train.ReplayTime{1}=k*LTT_Data_Train.ReplayTime{1};
    LTT_Data_Train.ReplayTime{2}=k*LTT_Data_Train.ReplayTime{2};
    %% CPD Warp
    disp('Please double check which robot motion needs to be warped!');
    fig2_handle = figure(2);
    set(fig2_handle,'position', [962 562 958 434]);
    rigidCompensate = 0;  %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    [Z_Data_Test2, warp] = LTT_Warping_CPD(LTT_Data_Train, points_W(:,1:2), points_Test_W(:,1:2), si , '2D', WarpIndex, rigidCompensate);
    fig3_handle = figure(3);
    set(fig3_handle,'position', [962 562 958 434]);
    orig_fig = subplot(1,2,1); scatter(points_W(:,1), points_W(:,2),'r*'); title('Train');
    warp_fig = subplot(1,2,2); scatter(points_Test_W(:,1), points_Test_W(:,2),'r*'); title('Test');
    draw_grid([-0.5 0.8], [1 -0.7], warp, 20, orig_fig, warp_fig)
    subplot(orig_fig); axis equal;xlim([-0.5,1]); ylim([-0.7,0.8]); drawnow;
    subplot(warp_fig); axis equal; xlim([-0.5,1]); ylim([-0.7,0.8]); drawnow;
    %% Run CFS % display an animation of fanuc robot following designed traj -WR
    disp('======================================================================')
    %Remember to check collision if workpiece location is chaged substantially!
    disp('======================================================================');
    fig1_handle = figure(2);
    set(fig1_handle,'position', [962 42 958 434]);
    % save as LTT_Data_UCBtest for CFS check
    LTT_Data_UCBtest = Z_Data_Test2;
    save('F:\TeTang\V4.0\CFS_ver1.1\dataLTT\UCBTest.mat', 'LTT_Data_UCBtest');
    % CFS check
    CFS_Main
    input('Use CFS toolbox to check collision. Press any key to execute the motion!!!')
    %% Run the robot
    Traj_Download_Run(Z_Data_Test2, si, 'Download', 'Run');
    opt = {}; opt.is_dual_arm = true; opt.robot_idx = WarpIndex;
    %     if RopeStep == 4
    %         PosCmd = [[0,0,0,0,-90,0];[0,0,0,0,-90,0]];
    %     end
    %     tp_pos_run(si, PosCmd(WarpIndex,:), opt);
end

%% Brake on
wasBrakeOff = brake_on_off(si.ParamSgnID, 'on');
wasStopped = tg_start_stop('stop');
