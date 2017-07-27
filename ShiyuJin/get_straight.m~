%%  setuo matlab ros after starting a ros in ubuntu
setenv('ROS_MASTER_URI','http://192.168.1.40:11311')                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
setenv('ROS_IP','192.168.1.90')
rosinit()

sub=rossubscriber('tracker/object');

%% 
received_data = receive(sub,3);    % receive current position
points = received_data.Rope.Nodes;
N = size(points,1);   % number of capsules
length_rope = (N+3)*0.02;    % +3 is mannally coded
origin = [470 0 180 -180 0 0];  % robot origin position
origin_joint = [0 0 0 0 -90 0 ];
%% 
%Brake Off
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(si.ParamSgnID, 'off');
%%
tip_index1 = 2;   % end of rope , pick index
tip_index2 = N-1;
if points(2).X * points(N-1).X < 0   % if two ends of rope is in both side of center 
    [pick_point(1,:), robot_move(1)] = PickPoint(points, tip_index1, si); 
    [pick_point(2,:), robot_move(2)] = PickPoint(points, tip_index2, si);
else   % scenario 2, two ends is in same side
    if abs(points(tip_index1).X) < abs(points(tip_index2).X)   %find which end of rope is more closer to center
        index_move_to_center = tip_index1;
    else
        index_move_to_center = tip_index2;
    end
    [pick_point, robot_move] = PickPoint(points, index_move_to_center, si);

    higher_pick_point = pick_point;   %
    higher_pick_point(3) = higher_pick_point(3) + 100  % first go to the point higher than pick point 100mm

    if robot_move == 1  %robot_move is the robot that moves, 1 is the right robot, 2 is left one
        center_point = [270,-640,-200,-180,0,179];  
    else
        center_point = [270,640,-200,-180,0,179];
    end
    
    %%% change pick point angle
    if abs(pick_point(6) - center_point(6)) > 180  % if the turning angle is larger than 180, make it 360 - angle
        if pick_point(6) > 0
            pick_point(6) = pick_point(6) - 180;
            higher_pick_point(6) = higher_pick_point(6) - 180;
            center_point(6) = center_point(6) + 180;
        else
            pick_point(6) = pick_point(6) + 180;
            higher_pick_point(6) = higher_pick_point(6) + 180;
            center_point(6) = center_point(6) - 180;
        end
    end
                
    pick_point_joint= fanucikine(pick_point, si.ri{robot_move}, origin_joint );  % convert xyzwpr to 6 joints' position
    higher_pick_point_joint= fanucikine(higher_pick_point, si.ri{robot_move}, origin_joint );
    center_point_joint = fanucikine(center_point, si.ri{robot_move}, origin_joint );

    Test = {}; % template input for download
    Test.DesJntPos{robot_move} = [origin_joint;  % joint position
        higher_pick_point_joint;
        pick_point_joint;
        pick_point_joint;
        pick_point_joint;
        higher_pick_point_joint;
        center_point_joint;
        center_point_joint;
        center_point_joint;
        origin_joint];
    Test.DesJntPos{3-robot_move} = [origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint];
    Test.GrpCmd{robot_move} = [ 0,0,0,0,0,1;  %a gripper command 
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    3,0,0,1,0,1;  % hold
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
     0,0,0,1,0,1; % release
     0,0,0,0,0,1;
     0,0,0,0,0,1];
    Test.GrpCmd{3-robot_move} = [0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1];
    Test.ReplayTime{1} =3*[1;1;1;1;1;3;1;1;1];  % time for each precess
    Test.ReplayTime{2} = 3*[1;1;1;1;1;3;1;1;1];

    disp('======================================================================')
    fig1_handle = figure(1);
    set(fig1_handle,'position', [962 42 958 434]);

    % save as LTT_Data_UCBtest for CFS check
    LTT_Data_UCBtest = Test;
    save('F:/TeTang/V4.0/CFS_ver1.1/dataLTT/UCBTest.mat', 'LTT_Data_UCBtest');

    % CFS check
    CFS_Main
    input('Use CFS toolbox to check collision. Press any key to execute the motion!!!')

    Traj_Download_Run(Test, si, 'Download', 'Run');

    % receive data again to prepare to make rope straight
    
    received_data = receive(sub,3);
    points = received_data.Rope.Nodes;
    former_robot = robot_move;
    [pick_point(1,:), robot_move(1)] = PickPoint(points, index_move_to_center, si, 3-former_robot);
    [pick_point(2,:) robot_move(2)] = PickPoint(points, N+1-index_move_to_center, si, former_robot);
end

for index = 1:2
    
    higher_pick_point = pick_point(index,:);
    higher_pick_point(3) = higher_pick_point(3) + 100  % go to a higher point 
    
    if robot_move(index) == 1
        drop_point = [370,length_rope/2*1000-635,-200,-180,0,90];  % 635 is hard coded, the distance between two robot  about 635mm*2
    else
        drop_point = [370,635-length_rope/2*1000,-200,-180,0,90];
    end
    
    if abs(pick_point(index,6) - drop_point(6)) > 180  % if angle >180 make it less than 180
        if pick_point(index,6) > 0
            pick_point(index,6) = pick_point(index,6) - 180;
            higher_pick_point(6) = higher_pick_point(6) - 180;
            drop_point(6) = drop_point(6) + 180;
        else
            pick_point(index,6) = pick_point(index,6) + 180;
            higher_pick_point(6) = higher_pick_point(6) + 180;
            drop_point(6) = drop_point(6) - 180;
        end
    end
    
    pick_point_joint= fanucikine(pick_point(index,:), si.ri{robot_move(index)}, origin_joint );  % code below share same comment as above code
    higher_pick_point_joint= fanucikine(higher_pick_point, si.ri{robot_move(index)}, origin_joint );
    drop_point_joint = fanucikine(drop_point, si.ri{robot_move(index)}, origin_joint )
    Test.DesJntPos{robot_move(index)} = [origin_joint; % a vector of all the joint angles in a single step
        higher_pick_point_joint;
        pick_point_joint;
        pick_point_joint;
        pick_point_joint;
        higher_pick_point_joint;
        drop_point_joint;
        drop_point_joint;
        drop_point_joint;
        origin_joint];
    Test.GrpCmd{robot_move(index)} = [ 0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    3,0,0,1,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
     0,0,0,1,0,1;
     0,0,0,0,0,1;
     0,0,0,0,0,1];
    Test.ReplayTime{robot_move(index)} = 3*[1;1;1;1;1;3;1;1;1];
end
%gripper_action(si.ParamSgnID(robot_move),'DRIVE');
disp('======================================================================')
    fig1_handle = figure(1);
    set(fig1_handle,'position', [962 42 958 434]);

    % save as LTT_Data_UCBtest for CFS check
    LTT_Data_UCBtest = Test;
    save('F:/TeTang/V4.0/CFS_ver1.1/dataLTT/UCBTest.mat', 'LTT_Data_UCBtest');

    % CFS check
    CFS_Main
    input('Use CFS toolbox to check collision. Press any key to execute the motion!!!')

    Traj_Download_Run(Test, si, 'Download', 'Run');



%% 

% Brake On
wasBrakeOff = brake_on_off(si.ParamSgnID, 'on');
wasStopped = tg_start_stop('stop');
