%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%  Find the closest grasping node on rope for TSM-RPM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Rui Wang, 08/03/2017       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [closestPts, ManOrNot, picIdx, stepBegin, gotoinit] = FindClosestPts(LTT_Data_Train, WarpIndex, points_W) % points_W is an array of rope pics

% set grip state representatives
gripKeep  = [0 0 0 0 0 1]; % under this state, the grip does not change
gripClose = [3 0 0 1 0 1]; % under this state, the grip closes
gripOpen  = [0 0 0 1 0 1]; % under this state, the grip opens

totalSteps = size(LTT_Data_Train.GrpCmd{1}, 1);
gotoinit{1} = zeros(totalSteps, 1); % set to 0 by default
gotoinit{2} = zeros(totalSteps, 1);
% define rope and gripper status in each step
for idx = WarpIndex
    [~, GripCloseSteps] = ismember(gripClose, LTT_Data_Train.GrpCmd{idx}, 'rows'); % find steps where gripper closes
    [~, GripOpenSteps]  = ismember(gripOpen,  LTT_Data_Train.GrpCmd{idx}, 'rows'); % find steps where gripper opens
    grpState = 0; % gripper is open(0) by default; 0: the gripper clenches
    cnt = 1; % counting the index of the pic of the current rope
    picIdx = []; % stores the index of the pic of the current rope
    % scan through all steps and get robot motion states!
    for step = 1 : totalSteps
        if LTT_Data_Train.GrpCmd{idx}(step, :) == gripClose % if gripper closes at this step
            grpState = 1;
            cnt = cnt + 1;
        end
        if LTT_Data_Train.GrpCmd{idx}(step, :) == gripOpen % gripper opens
            grpState = 0;
            %cnt = cnt + 1;
        end
        picIdx(step) = cnt; % which pic should be followed, if is grasping
        gripOrNot{idx}(step) = grpState; % if == 1, the gripper is closed
        if step > 1 && norm(LTT_Data_Train.TCP_xyzwpr_W{idx}(step, 1:2) - LTT_Data_Train.TCP_xyzwpr_W{idx}(step - 1, 1:2)) < 50 ...
            || idx == 1 && step == 1 && norm(LTT_Data_Train.TCP_xyzwpr_W{idx}(step, 1:2) - [680, 635]) < 10 ...
            || idx == 2 && step == 1 && norm(LTT_Data_Train.TCP_xyzwpr_W{idx}(step, 1:2) - [680, -637]) < 10
            % if the robot is not moving at all
            % Note: only applicable for robots of current state!
            MvOrNot{idx}(step) = 0;
        else
            MvOrNot{idx}(step) = 1;
        end
    end
    % Warning! As the gripper closes at the end of the step where the
    % gripper is set to close, it is not closed along the way. So gripper
    % state needs modification. It is not closed on the way!
    for step = 0 : totalSteps-2
        step = totalSteps - step;% must search in the reverse direction or all gripOrNot will be set to 0
        if gripOrNot{idx}(step) == 1 && gripOrNot{idx}(step-1) == 0
            gripOrNot{idx}(step) = 0;
        end
    end
    gripOrNot{idx}(1) = 0;
    % Warning! As the gripper opens at the end of the step, it is still closed along the way. So gripper
    % state needs modification.
    for step = 2 : totalSteps
        if gripOrNot{idx}(step) == 0 && gripOrNot{idx}(step-1) == 1 && MvOrNot{idx}(step) == 1
            gripOrNot{idx}(step) = 1;
        end
    end

    gripOrNot{1}(1) = 0; gripOrNot{2}(1) = 0; % first step is always not gripping
    ManOrNot{idx} = 0; % all steps default to zero | if == 1, the robot is manipulating the rope, i.e. grasping & moving it
    for step = 1 : totalSteps
        if MvOrNot{idx}(step) == 1 && gripOrNot{idx}(step) == 1 % is gripping and moving rope
            ManOrNot{idx}(step) = 1;
        elseif MvOrNot{idx}(step) == 1 && gripOrNot{idx}(step) == 0 % is moving to a target node
            ManOrNot{idx}(step) = -1;
        else ManOrNot{idx}(step) = 0; % not moving
        end
    end
    % If ManOrNot{idx}(step) == 1, the (x, y) coord of gripper should be
    % calculated by warping in tangent space & integration
end

% find the training rope pic for each step
totalSteps = size(LTT_Data_Train.GrpCmd{1}, 1);
picIdx = ones(totalSteps, 1);
picCnt = 1; % counting which pic of rope is the curr rope
stepBegin(picCnt) = 1;
for step = 1 : totalSteps
    if ManOrNot{1}(step) == 1 || ManOrNot{2}(step) == 1
        picCnt = picCnt + 1;
        stepBegin(picCnt) = step + 1;
    end
    picIdx(step) = picCnt;
end
stepBegin(end) = totalSteps;

closestPts = {zeros(totalSteps, 1), zeros(totalSteps, 1)};
for idx = WarpIndex
    % now find point on training rope closest to gripping point
    for step = 1 : totalSteps
        closestPts{idx}(step, :) = dsearchn(points_W{picIdx(step)}(:, 1:2)*1000, ...
        LTT_Data_Train.TCP_xyzwpr_W{idx}(step, 1:2)); % built-in matlab func for closest pt
        % points_W{num} stores the whole rope, for robot No.idx's No.num step
        if norm( points_W{picIdx(step)}(closestPts{idx}(step),1:2)*1000 - LTT_Data_Train.TCP_xyzwpr_W{idx}(step, 1:2) ) > 50
            gotoinit{idx}(step) = 1; % if the arm is trained to avoid occluding the rope, go to initial position
        end
    end
    % Now closestPts{idx}(i) is the index of rope node closest to grasping
    % point if step i is grasping step; otherwise it's 0
end
end