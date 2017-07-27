% Wenjie Chen, FANUC Corporation, 2016/08/26
% To transform camera point cloud data to world (and base) frame coordinates
% Also filter out the point cloud outside the specified region

function camData = camPCL2World(si, camData, region, opt)

if nargin < 3,  region = [];  end
if nargin < 4,  opt = [];  end
if ~isfield(opt,'robot_idx') || isempty(opt.robot_idx),  opt.robot_idx = 0;  end   % robot index which base frame the point cloud data is transformed to

if ~strcmpi(si.camera_type, 'ensenso')
    error('Currently only ensenso camera is calibrated with workspace!');
end

if size(region,1) ~= 2,  region = region';  end
if size(region,1) ~= 2 || size(region,2) ~= 3
    error('Please specify the region as a 2x3 matrix (the xyz limits of a cubic region)!');
end

% transform to world frame and filter into the specified region
xyz = camData.ptCld.Location/1000;  % change to m
xyz = xyz + repmat(si.CalibGridCenter_W, size(xyz,1), 1);
if ~isempty(region)
    valid_idx_i = cell(1,3);
    for i = 1:3
        valid_idx_i{i} = xyz(:,i) >= region(1,i) & xyz(:,i) <= region(2,i);
    end
    valid_idx = valid_idx_i{1} & valid_idx_i{2} & valid_idx_i{3};
    xyz = xyz(valid_idx,:);
end
camData.xyz_w = xyz;

% transform to robot base frame if needed
if opt.robot_idx ~= 0
    num = numel(opt.robot_idx);
    camData.robot_idx = opt.robot_idx;
    camData.xyz_b = zeros(size(xyz,1), 3*num);
    for rn = 1:num
        for i = 1:size(xyz, 1)
            tmp = si.ri{opt.robot_idx(rn)}.T_B2W \ [xyz(i,:), 1]';
            camData.xyz_b(i, rn*3-2:rn*3) = tmp(1:3)';
        end        
    end
end

end