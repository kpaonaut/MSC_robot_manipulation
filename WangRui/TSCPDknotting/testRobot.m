    sub = rossubscriber('tracker/object'); % your PC, as ros master, should be publishing this topic (tracked obj) now
    received_data = receive(sub,3); % wait for at most 3 seconds.
    points = received_data.Rope.Nodes; % N*1 vector, each element: .X, .Y, .Z
    LENGTH = callen([points.X]', [points.Y]');
    % transform test rope to world frame
    points_Test_U = [[points.X]', [points.Y]', [points.Z]'];
    points_Test_W = transformU2W(points_Test_U, si);
    if (step == 1) && (judgeOrder(points_Test_W) == 1)
        tf = 1; % needs to reverse all received data hereafter!
    end
    if tf == 1
        points_Test_W(:, 1) = flipud(points_Test_W(:, 1));
        points_Test_W(:, 2) = flipud(points_Test_W(:, 2));
    end

j=1;
stepBegin = 1;
stepEnd = 2;
idx = 2;
num = 44;
LTT_Data_Test = LTT_Data_Train;
LTT_Data_Test.TCP_xyzwpr_W{idx}(1, 1:2) = points_Test_W(num, 1:2) * 1000; % unit must be mm!    
LTT_Data_Test.TCP_xyzwpr_W{idx}(1, 3) = 85;
LTT_Data_Test.TCP_xyzwpr_W{idx}(2, 3) = 150;

for j=1:2
LTT_Data_Test.TCP_T_W{idx}(:, :, j) = xyzwpr2T(LTT_Data_Test.TCP_xyzwpr_W{idx}(j, :));
LTT_Data_Test.TCP_T_B{idx}(:, :, j) = FrameTransform(LTT_Data_Test.TCP_T_W{idx}(:, :, j), 'T', 'W2B', si, idx);
LTT_Data_Test.TCP_xyzwpr_B{idx}(j, :) = T2xyzwpr(LTT_Data_Test.TCP_T_B{idx}(:, :, j));
LTT_Data_Test.DesJntPos{idx}(j, :) = fanucikine(LTT_Data_Test.TCP_xyzwpr_B{idx}(j, :), si.ri{idx}, LTT_Data_Train.DesJntPos{idx}(j, :));    LTT_Data_Test.DesJntPos{idx} = LTT_Data_Test.DesJntPos{idx}(stepBegin:stepEnd, :);
end
LTT_Data_Test.ReplayTime{idx} = LTT_Data_Test.ReplayTime{idx}(stepBegin:stepEnd-1, :);
LTT_Data_Test.GrpCmd{idx} = LTT_Data_Test.GrpCmd{idx}(stepBegin:stepEnd, :);
LTT_Data_Test.ReplayTime{1} = LTT_Data_Test.ReplayTime{1}(stepBegin:stepEnd-1, :);
LTT_Data_Test.GrpCmd{1} = LTT_Data_Test.GrpCmd{1}(stepBegin:stepEnd, :);

%%
    fig1_handle = figure(2);
    set(fig1_handle, 'position', [962 42 958 434]);
    % save as LTT_Data_UCBtest for CFS check
    LTT_Data_UCBtest = LTT_Data_Test;
    save('F:\TeTang\V4.0\CFS_ver1.1\dataLTT\UCBTest.mat', 'LTT_Data_UCBtest');
    % CFS check
    CFS_Main
%%
    wasStopped = tg_start_stop('start');
    wasBrakeOff = brake_on_off(si.ParamSgnID, 'off');
    % Run!
    Traj_Download_Run(LTT_Data_Test, si, 'Download', 'Run');
    % Brake on
    wasBrakeOff = brake_on_off(si.ParamSgnID, 'on');
    wasStopped = tg_start_stop('stop');