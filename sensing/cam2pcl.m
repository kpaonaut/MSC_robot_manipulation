% Wenjie Chen, FANUC Corporation, 2016/01/15
% Take camera capture images and do image thresholding to get cable point
% clouds and label the point clouds with corresponding types
% 2016/05/19 modified to include processing for Ensenso cameras

function cableData = cam2pcl(imThOpt, k2, save_enb, name_enb, data_path)
if nargin < 1, imThOpt = []; end
if nargin < 2,  k2 = [];  end
if nargin < 3,  save_enb = 0;  end
if nargin < 4,  name_enb = 0;  end
if nargin < 5,  data_path = '../data/old/';  end

%% Get camera data
opt = [];
% opt.datapath = data_path;
% opt.savedata = 1;
% opt.plot_enb = 1;
% opt.savefig = 1;

if isempty(k2)  % Ensenso camera case
    camData = GetEnsensoData(opt);
else    % Kinect V2
    opt.types = 'cd';   % color and depth outputs
    opt.register = 1;
    camData = GetKinectData(k2, opt);
end

%% Image thresholding
%%% !!!!!!!!!!!! red
%imThOpt.type = 'orange';
%imThOpt.plot_enb = 1;
% imThOpt.downsample = 1;
% imThOpt.datapath = data_path;
% imThOpt.time = camData.time;
% imThOpt.savefig = 1;
% imThOpt.savedata = 1;

if isempty(k2)  % Ensenso camera case
    imThOpt.device = 'ensenso';
    [ptCld, fh] = imgThreshold(camData.rgba, camData.xyz, imThOpt);
else    % Kinect V2
    imThOpt.device = 'kinect';
    [ptCld, fh] = imgThreshold(camData.color, camData.depth_hd_rect, imThOpt);
end

%% Save point cloud data
if name_enb
    Type_set = {'doublefold', 'fold', 'line', 'tie'};
    type_idx = input('What is the cable type? [1-doublefold, 2-fold, 3-line, 4-tie] : ');
    type_name = Type_set{type_idx};
else
    type_name = 'unknown';
end

if save_enb
    filepath = [data_path, type_name, '/'];
    if isempty(ls(filepath)),  mkdir(filepath);  end
    save([filepath, camData.time, '_camData.mat'], 'camData', 'ptCld', 'type_name');  
    if ~isempty(fh),  saveas(fh, [filepath, camData.time, '_img.png']);  end
end

cableData.ptCld = ptCld;
cableData.type_name = type_name;

end
