function [closestPts, MoveOrNot] = FindClosestPts(LTT_Data_Train, WarpIndex, points_W)
% first set grip state representatives
gripKeep  = [0 0 0 0 0 1]; % under this state, the grip does not change
gripClose = [3 0 0 1 0 1]; % under this state, the grip closes
gripOpen  = [0 0 0 1 0 1]; % under this state, the grip opens
totalSteps = size(LTT_Data_Train.GrpCmd{1}, 1);
for idx = WarpIndex
    [~, GripCloseSteps] = ismember(gripClose, LTT_Data_Train.GrpCmd{idx}, 'rows'); % find steps where gripper closes
    [~, GripOpenSteps] = ismember(gripOpen, LTT_Data_Train.GrpCmd{idx}, 'rows'); % find steps where gripper opens
    grpState = 0; % gripper is open(0) by default; 0: the gripper clenches
    cnt = 0; % counting the index of the pic of the current rope
    picIdx = []; % stores the index of the pic of the current rope
    for step = 1 : totalSteps
        if ismember(step, GripCloseSteps) % if gripper closes at this step
            grpState = 1;
            cnt = cnt + 1;
        end
        if ismember(step, GripOpenSteps) % gripper opens
            grpState = 0;
            cnt = cnt + 1;
        end
        picIdx(step) = cnt; % which pic should be followed, if is grasping
        gripOrNot{idx}(step) = grpState;
    end
    MoveOrNot{idx} = 0; % all steps default to zero
    for step = 2 : totalSteps
        prevxyz = LTT_Data_Train.TCP_xyzwpr_W{idx}(step, 1:3);
        xyz = LTT_Data_Train.TCP_xyzwpr_W{idx}(step, 1:3);
        if (norm(xyz - prevxyz) > 10) and (gripOrNot{idx}(step) = 1) % is gripping and moving rope
            MoveOrNot{idx}(step) = 1;
        end
    end
    % If MoveOrNot{idx}(step) == 1, the (x, y) coord of gripper should be
    % calculated by warping in tangent space & integration
    
    % now find point on training rope closest to gripping point
    for step = 1 : totalSteps
%         num = picIdx(step); % the index of the stored rope for gripper of this state to compare with
%         closestPts{idx}(step, :) = dsearchn(points_W{idx, num}, ...
%             delaunayTriangulation(points_W{idx, num}), ...
%             LTT_Data_Train.TCP_xyzwpr_W{idx}(step, 1:2)); % built-in matlab func for closest pt
        num = picIdx(step); % the index of the stored rope for gripper of this state to compare with
        if MoveOrNot{idx}(step) == 1
            points_W = points_W_Goal;
        end
        closestPts{idx}(step, :) = dsearchn(points_W, ...
            delaunayTriangulation(points_W), ...
        LTT_Data_Train.TCP_xyzwpr_W{idx}(step, 1:2)); % built-in matlab func for closest pt
        
        % points_W{idx, num} stores the whole rope, for robot No.idx's
        % No.num step
    end
    % Now closestPts{idx}(i) is the index of rope node closest to grasping
    % point if step i is grasping step; otherwise it's 0
end
end