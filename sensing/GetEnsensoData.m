function camData = GetEnsensoData(opt)
% Input arguments:
%       opt     - options with following elements:
%           plot_enb    - 1: enable plots; 0 - disable plots
%           savefig     - 1: save plot to png file; 0: not save (default)
%           savedata    - 1: save data to mat file; 0: not save (default)
%           datapath    - path to save data (default: './data/')
%
% Output arguments:
%       camData  - structure for Ensenso camera data with following elements:
%                   camData.time (capture timestamp: yyyyMMdd_HHmmssMS)
%                   camData.xyz (32 bit float, XYZ values, mm, workspace coordinates)
%                   camData.rgba (8 bit uint, RGB + alpha values, alpha =
%                   255 for valid points, alpha = 0 for invalid points, 
%                   camData.rgb (8 bit uint, RGB values)
%
% Wenjie Chen, FANUC Corporation, 2016/05/19

if nargin < 1,  opt = [];  end

% default: do not plot images
if ~isfield(opt,'plot_enb') || isempty(opt.plot_enb),  opt.plot_enb = 0;  end 
% default: do not save images
if ~isfield(opt,'savefig') || isempty(opt.savefig),  opt.savefig = 0;  end 
% default: do not save data
if ~isfield(opt,'savedata') || isempty(opt.savedata),  opt.savedata = 0;  end 
% default: save to './data/'
if ~isfield(opt,'datapath') || isempty(opt.datapath),  opt.datapath = '../data/';  end 
% default: colorScale = 1
if ~isfield(opt,'colorScale') || isempty(opt.colorScale),  opt.colorScale = 1;  end 

PrintMsg('peek');

tic;  [camData.xyz, camData.rgba] = nxGetPtCld();  camData.rgb = nxGetRGBonly();  tElapsed = toc;
camData.xyz = permute(camData.xyz,[3,2,1]);
camData.rgba = permute(camData.rgba,[3,2,1]);
camData.rgb = permute(camData.rgb,[3,2,1]);
PrintMsg('tElapsed', num2str(tElapsed));

camData.time = char(datetime('now','Format','yyyyMMdd_HHmmssMS'));
camData.device = 'ensenso';

if opt.savedata,  save([opt.datapath, camData.time, '_camData.mat'], 'camData');  end

if opt.plot_enb
    color = imresize(camData.rgb(:,:,1:3),opt.colorScale);
    figure; imshow(color);
    title('Color Source', 'fontsize', 14)
    if opt.savefig,  imwrite(color, [opt.datapath, camData.time, '_color.png']);  end

    pcld = pointCloud(camData.xyz, 'Color', camData.rgba(:,:,1:3));
    figure; pcshow(pcld);   
    title('Point Cloud', 'fontsize', 14)
    if opt.savefig,  saveas(gcf, [opt.datapath, camData.time, '_pcl.png']);  end
    
%     closefigs(pauseTime);
end

end