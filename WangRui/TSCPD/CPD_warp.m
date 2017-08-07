%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Warp LTT_Data by CPD function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 04/17/2017       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [LTT_Data_Test, warp] = CPD_warp(LTT_Data_Train, PtCld_Train, PtCld_Test, si, dim, robot_idx, rigidCompensate)


%% specify which arm to warp
if nargin < 6
    robot_idx = [1,2];
end
    
%% CPD
X = PtCld_Test;
Y = PtCld_Train;
% rigid rotation first
if rigidCompensate == true
    opt = [];
    opt.method = 'rigid'; % use rigid registration
    opt.fgt = 0;
    opt.viz = 1;          % show every iteration
    opt.rot = 1;
    opt.normalize = 0;
    opt.scale = 0;
    %registering Y to X
    Transform_rigid = cpd_register(X,Y,opt);
    %generate warp function handle
    warp_rigid = @(x) (cpd_transform(x', Transform_rigid))';
    Y = Transform_rigid.Y;
end

% non rigid 
% Set the options
opt = [];
opt.method = 'nonrigid'; % whether or not to use rigid registration
opt.fgt = 0;
opt.viz = 1;          % show every iteration
opt.beta = 1;      %(default 2) Gaussian smoothing filter size. Forces rigidity.
opt.lambda = 25;    %(default 3) Regularization weight. 
% registering Y to X
Transform = cpd_register(X,Y,opt);
% generate warp function handle
warp = @(x) (cpd_transform(x', Transform))';

if rigidCompensate == true
    warp = @(x) warp(warp_rigid(x));
end

%% create New_LTT_Data after warping
LTT_Data_Test = LTT_Data_Train;
LTT_Data_Test.PtCld = PtCld_Test;
for i = robot_idx
    for j = 1:size(LTT_Data_Train.TCP_T_W{i},3)
        if strcmp(dim, '3D')
            LTT_Data_Test.TCP_T_W{i}(:,:,j) = TPS_warp_T(LTT_Data_Train.TCP_T_W{i}(:,:,j), warp);
        elseif strcmp(dim, '2D')
            LTT_Data_Test.TCP_T_W{i}(:,:,j) = TPS_warp_T_2D(LTT_Data_Train.TCP_T_W{i}(:,:,j), warp);
        end
        LTT_Data_Test.TCP_xyzwpr_W{i}(j,:) = T2xyzwpr(LTT_Data_Test.TCP_T_W{i}(:,:,j));
        LTT_Data_Test.TCP_T_B{i}(:,:,j) = FrameTransform(LTT_Data_Test.TCP_T_W{i}(:,:,j), 'T', 'W2B', si, i);
        LTT_Data_Test.TCP_xyzwpr_B{i}(j,:) = T2xyzwpr(LTT_Data_Test.TCP_T_B{i}(:,:,j));
        % Joint Position
        LTT_Data_Test.DesJntPos{i}(j,:) = fanucikine(LTT_Data_Test.TCP_xyzwpr_B{i}(j,:), si.ri{i}, LTT_Data_Train.DesJntPos{i}(j,:));
    end
end
