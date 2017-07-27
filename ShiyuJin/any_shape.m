%%  setuo matlab ros after starting a ros in ubuntu,only run once
setenv('ROS_MASTER_URI','http://192.168.1.40:11311')                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
setenv('ROS_IP','192.168.1.90')
rosinit()

sub=rossubscriber('tracker/object');
%% 

clear anggg2   % clear a lot of thing for second time sketching
clear downsample_x
clear downsample_y
clear line_curv
clear anggg
clear anggg1
clear any_shape
close all

origin_joint = [0 0 0 0 -90 0 ];   % robot jointorigin
received_data = receive(sub,3);
points = received_data.Rope.Nodes;
N = size(points,1);


for i = 1:10000  %interpolation, because drawing lines may be faster at some points and smaller at other points
    Dis=sqrt(diff(custom_x).^2+diff(custom_y).^2);
    [max_value,max_index] = max(Dis); 
    custom_x = [custom_x(1:max_index); 0.5*(custom_x(max_index) + custom_x(max_index+1)); custom_x(max_index+1:end)];
    custom_y = [custom_y(1:max_index); 0.5*(custom_y(max_index) + custom_y(max_index+1)); custom_y(max_index+1:end)];
end

index_downsample = size(custom_x,1)/N;
total_length = 0;

for i = 1:N
    index = round(index_downsample*i);
    downsample_x(i) = custom_x(index);
    downsample_y(i) = custom_y(index);  
end

for i = 1:N-1 
        total_length = total_length + ((downsample_x(i+1)-downsample_x(i))^2 + (downsample_y(i+1)-downsample_y(i))^2)^0.5;
       
end
distance_downsample = total_length/(0.02*N);
%down sample distance
downsample_x = downsample_x/distance_downsample;
downsample_y = downsample_y/distance_downsample;
downsample_x  = downsample_x -  mean(downsample_x);
downsample_y  = downsample_y -  mean(downsample_y);


for i = 1:N
    any_shape(i).X = -downsample_x(i);  % rotate sketching 180 degree
    any_shape(i).Y = -downsample_y(i);
    any_shape(i).Z = 0.12;
end

heart = any_shape;

for i = 1:N-1
    line_curv(i) = atan( (downsample_y(i+1)-downsample_y(i)) / (downsample_x(i+1)-downsample_x(i)) )/pi*180; % calculate curvature
end
anggg = diff(line_curv);
anggg1 = abs(abs(anggg)-90);
anggg2 = [90,90,90,90,90,anggg1(5:end-4),90,90,90,90,90];  % do not choose first and last 5 index
pick_sequence1 = [];
guaidian =  input('Input number of turning points!!!!!!!!');
for i = 1:guaidian
    [min_ang,min_index] = min(anggg2);
    pick_sequence1(i) = min_index;
    for j = 1:9
        anggg2(min_index+5-j) = 90;  % if on index is chosen, then its neighbours won't be chosen later
    end 
end
pick_sequence1 = [pick_sequence1, 1, N];
figure(11)
set(gcf, 'Position', [600, 500, 1000, 500])
axis([-0.6 0.6 -0.3 0.3])
scatter(downsample_x,downsample_y)
for i = pick_sequence1
    hold on
    scatter(downsample_x(i),downsample_y(i),'r')
end
hold off
pick_sequence1

if any_shape(1).X > any_shape(N).X
    pick_sequence1 = [sort(pick_sequence1(1:guaidian)), 1, N]; % pick index with an increasing sequence
else
    pick_sequence1 = [sort(pick_sequence1(1:guaidian), 'descend'), 1, N];
end
if points(1).X < points(N).X
    pick_sequence1 = N+1-pick_sequence1;
end
pick_sequence1

%%

%% 

Test = {};% template trajectory
    Test.DesJntPos{1} = [origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint];
    Test.DesJntPos{2} = [origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint;
        origin_joint];
    Test.GrpCmd{1} = [ 0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
     0,0,0,0,0,1;
     0,0,0,0,0,1;
     0,0,0,0,0,1];
    Test.GrpCmd{2} = [0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1];
    Test.ReplayTime{1} = 3*[1;1;1;1;1;1;1;1;1];
    Test.ReplayTime{2} = 3*[1;1;1;1;1;1;1;1;1];
%% 
%Brake Off
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(si.ParamSgnID, 'off');
%% 

first_time = 1; % first movement only use one robot arm
for i =1:size(pick_sequence1,2)
    received_data = receive(sub,3);
    points = received_data.Rope.Nodes;

    pick_index = pick_sequence1(i);

    if first_time
        first_time = 0;
        if abs(points(pick_index).X-heart(pick_index).X) > abs(points(pick_index).X-heart(N+1-pick_index).X) 
            inverse_heart = 1;  % use inverse sequence of rope because it's closer to sketching
        else
            inverse_heart = 0;
        end
    end
    if inverse_heart == 1
        heart_index = N+1-pick_index;
    else 
        heart_index = pick_index;
    end
    
    if i == 2
        [pick_point, robot_move] = PickPoint(points, pick_index, si, 1);  % force it to use robot 1 , this is hard coded when recording video for presentation
    else
        [pick_point, robot_move] = PickPoint(points, pick_index, si);
    end
    [drop_point, robot_move] = PickPoint(heart, heart_index, si, robot_move);
    %pick
    higher_pick_point = pick_point;
    higher_pick_point(3) = higher_pick_point(3) + 40
    % make drop point higher
    drop_point(3) = drop_point(3) +40;
    
    pick_point
    drop_point
    if abs(pick_point(6) - drop_point(6)) > 180  % comments are same as get_straight.m file
        if pick_point(6) > 0
            pick_point(6) = pick_point(6) - 180;
            higher_pick_point(6) = higher_pick_point(6) - 180;
            drop_point(6) = drop_point(6) + 180;
        else
            pick_point(6) = pick_point(6) + 180;
            higher_pick_point(6) = higher_pick_point(6) + 180;
            drop_point(6) = drop_point(6) - 180;
        end
    end
    pick_point
    drop_point

    pick_point_joint= fanucikine(pick_point, si.ri{robot_move}, origin_joint );
    higher_pick_point_joint= fanucikine(higher_pick_point, si.ri{robot_move}, origin_joint );
    drop_point_joint = fanucikine(drop_point, si.ri{robot_move}, origin_joint );

    Test1 = Test;
    Test1.DesJntPos{robot_move} = [origin_joint;
        higher_pick_point_joint;
        pick_point_joint;
        pick_point_joint;
        pick_point_joint;
        higher_pick_point_joint;
        drop_point_joint;
        drop_point_joint;
        drop_point_joint;
        origin_joint];
    Test1.GrpCmd{robot_move} = [ 0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    3,0,0,1,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
    0,0,0,0,0,1;
     0,0,0,1,0,1;
     0,0,0,0,0,1;
     0,0,0,0,0,1];
 
 
%hold the center    
    if i >=2 && i <= 4 % hard coded for recording video, robot manipulate rope collaborately
        [hold_point, robot_hold] = PickPoint(points, pick_sequence1(i-1), si, 3-robot_move);
        hold_point_joint= fanucikine(hold_point, si.ri{robot_hold}, origin_joint );
            Test1.DesJntPos{robot_hold} = [origin_joint;
                hold_point_joint;
                hold_point_joint;
                hold_point_joint;
                hold_point_joint;
                hold_point_joint;
                hold_point_joint;
                hold_point_joint;
                hold_point_joint;
                origin_joint];
            Test1.GrpCmd{robot_hold} = [ 0,0,0,0,0,1;
                    0,0,0,0,0,1;
                    0,0,0,0,0,1;
                    3,0,0,1,0,1;
                    0,0,0,0,0,1;
                    0,0,0,0,0,1;
                    0,0,0,0,0,1;
                     0,0,0,1,0,1;
                     0,0,0,0,0,1;
                     0,0,0,0,0,1];
    end
 disp('======================================================================')
    fig1_handle = figure(1);
    set(fig1_handle,'position', [962 42 958 434]);

    % save as LTT_Data_UCBtest for CFS check
    LTT_Data_UCBtest = Test1;
    save('F:/TeTang/V4.0/CFS_ver1.1/dataLTT/UCBTest.mat', 'LTT_Data_UCBtest');

    % CFS check
    CFS_Main
    input('Use CFS toolbox to check collision. Press any key to execute the motion!!!')

    Traj_Download_Run(Test1, si, 'Download', 'Run');

end
%% 

% Brake On
wasBrakeOff = brake_on_off(si.ParamSgnID, 'on');
wasStopped = tg_start_stop('stop');

    

