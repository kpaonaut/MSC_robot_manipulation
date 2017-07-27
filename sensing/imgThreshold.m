function [ptCloud, fh] = imgThreshold(rgb, xyz, opt)
% Input arguments: 
%       rgb             - rgb color image
%       depth           - depth image 
%       opt             - options with following elements:
%           device      - sensor device: 'ensenso'(default) or 'kinect'
%           type        - type for color thresholding (default: green)
%           time        - time of data update (default: current time)
%           plot_enb    - 1: enable plots; 0: disable plots (default)
%           savefig     - 1: save plot to png file; 0: not save (default)
%           savedata    - 1: save data to mat file; 0: not save (default)
%           datapath    - path to save data (default: './data/')
%
% Output argument:
%       ptCloud         - final point cloud after image thresholding
%
% Wenjie Chen, FANUC Corporation, 2016/01/13
% 2016/05/18, added processing for Ensenso camera and more color filters
% Modified by Te Tang, UC Berkeley, 09/28/2016, use function ColorThreshold_thresh to get color_mask

% default: ensenso
if ~isfield(opt,'device') || isempty(opt.device),  opt.type = 'ensenso';  end 
% default: green
if ~isfield(opt,'type') || isempty(opt.type),  opt.type = 'red';  end 
% default: 0
if ~isfield(opt,'downsample') || isempty(opt.downsample),  opt.downsample = 0;  end 
% default: green
if ~isfield(opt,'ds_size') || isempty(opt.ds_size), opt.ds_size = 500;  end 
% default: use the current time
if ~isfield(opt,'time') || isempty(opt.time),  opt.time = char(datetime('now','Format','yyyyMMdd_HHmmssMS'));  end 
% default: do not plot images
if ~isfield(opt,'plot_enb') || isempty(opt.plot_enb),  opt.plot_enb = 0;  end 
% default: do not save images
if ~isfield(opt,'savefig') || isempty(opt.savefig),  opt.savefig = 0;  end 
% default: do not save data
if ~isfield(opt,'savedata') || isempty(opt.savedata),  opt.savedata = 0;  end 
% default: save to './data/'
if ~isfield(opt,'datapath') || isempty(opt.datapath),  opt.datapath = './data/';  end 

switch opt.type
    case 'red'
        mask_h = {@(h) (h>=0.931)|(h<=0.017), @(s) (s>=0.119)&(s<=0.836), @(v) (v>=0.443)&(v<=0.792)};
    case 'blue'
        
    case 'yellow'
        
    case 'green'
        
    case 'lightblue'   
        
    case 'orange'
        
    case 'black'    % not stable
        mask_h = {@(h) (h>=0.143)&(h<=0.310), @(s) (s>=0.177)&(s<=0.754), @(v) (v>=0.051)&(v<=0.161)};
    case 'grey'     % not good
        
    case 'white'
        
    otherwise
        mask_h = {@(h) (h>0), @(s) (s>0), @(v) (v>0)};
end

if strcmpi(opt.device, 'ensenso')
    valid_mask = (rgb(:,:,4) == 255) & (~isnan(xyz(:,:,1)));
    rgb = rgb(:,:,1:3);
    xyz_w = xyz;
else    % kinect, here xyz is actually depth only
    outOfRange = 5000;
    valid_mask = (xyz > 0) & (xyz < outOfRange);
    xyz_w = depthToCloud(xyz);
end

hsv = rgb2hsv(rgb);
h = hsv(:,:,1);
s = hsv(:,:,2);
v = hsv(:,:,3);
%h = hsv(:,:,1)*360;
%s = hsv(:,:,2)*256;
%v = hsv(:,:,3)*256;

h_mask = mask_h{1}(h); 
s_mask = mask_h{2}(s); 
v_mask = mask_h{3}(v); 
color_mask = h_mask & s_mask & v_mask;

% [~, color_mask, h_mask, s_mask, v_mask] = ColorThreshold_thresh(rgb, HSV_min, HSV_max, false);

good_mask = color_mask & valid_mask;

% kinect should not take bwareaopen, since the depth data mapped to color
% is already very sparse (I deactivate the interpolation (time consuming) in GetKinectData.m)
if strcmpi(opt.device, 'ensenso')
    good_mask = bwareaopen(good_mask, 15);
end

good_xyz = zeros(sum(sum(good_mask)),3);
for i = 1:3
    tmpdata = xyz_w(:,:,i);
    good_xyz(:,i) = tmpdata(good_mask);
end

ptCloud = pointCloud(good_xyz);

% downsample point clouds
if opt.downsample
    gridStep = 3;   % resolution, mm
    ds_percentage = min(1, opt.ds_size / length(ptCloud.Location));
    ptCloud = pcdownsample(ptCloud, 'random', ds_percentage);
    ptCloud = pcdownsample(ptCloud, 'gridAverage', gridStep);
end

fh = -1;

if opt.plot_enb
    scrsz = get(groot,'ScreenSize');
    fh = figure('Position',[scrsz(3)*0.1 scrsz(4)*0.15 scrsz(3)*0.8 scrsz(4)*0.75]);
    
    subplot(2,3,1);
    imshow(good_mask);
    title('mask of detected object', 'FontSize', 14);
    
    subplot(2,3,2);
    pcshow(ptCloud, 'MarkerSize', 10);
    title('detected object point cloud', 'FontSize', 14);
    
    subplot(2,3,3);
    pcshow(pointCloud(xyz_w, 'Color', rgb));
    title('point cloud of full view', 'FontSize', 14);
    
    subplot(2,3,4);
    imshow(h_mask);
    title('h mask', 'FontSize', 14);
    
    subplot(2,3,5);
    imshow(s_mask);
    title('s mask', 'FontSize', 14);
    
    subplot(2,3,6);
    imshow(v_mask);
    title('v mask', 'FontSize', 14);
    
    if opt.savefig,  saveas(fh, [opt.datapath, opt.time, '_img.png']);  end
%     closefigs;
end

if opt.savedata,  save([opt.datapath, opt.time, '_pclData.mat'], 'rgb', 'xyz', 'ptCloud', 'good_mask', 'opt');  end

end
