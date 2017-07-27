function camData = GetKinectData(k2, opt)
% Input arguments:
%       k2      - Kinect 2 object
%       opt     - options with following elements:
%           types       - string containing characters as follows (default: cd):
%                         c - color;  d - depth;  i - infrared;  b - body
%           plot_enb    - 1: enable plots; 0 - disable plots
%           savefig     - 1: save plot to png file; 0: not save (default)
%           savedata    - 1: save data to mat file; 0: not save (default)
%           datapath    - path to save data (default: './data/')
%           register    - 1: register depth data to rgb (default); 0: not register
%
% Output arguments:
%       camData  - structure for Kinect 2 data with following elements:
%                   camData.color/depth/infrared/body/time/device
%
% Wenjie Chen, FANUC Corporation, 2015/12/18

if nargin < 2,  opt = [];  end

% default: output color and depth images
if ~isfield(opt,'types') || isempty(opt.types),  opt.types = 'cd';  end   
% default: do not plot images
if ~isfield(opt,'plot_enb') || isempty(opt.plot_enb),  opt.plot_enb = 0;  end 
% default: do not save images
if ~isfield(opt,'savefig') || isempty(opt.savefig),  opt.savefig = 0;  end 
% default: do not save data
if ~isfield(opt,'savedata') || isempty(opt.savedata),  opt.savedata = 0;  end 
% default: save to './data/'
if ~isfield(opt,'datapath') || isempty(opt.datapath),  opt.datapath = '../data/';  end 
% default: register = 0
if ~isfield(opt,'register') || isempty(opt.register),  opt.register = 0;  end 
% default: colorScale = 0.5
if ~isfield(opt,'colorScale') || isempty(opt.colorScale),  opt.colorScale = 0.5;  end 

c_enb = 0;  d_enb = 0;  i_enb = 0;  b_enb = 0;
if ~isempty(strfind(opt.types, 'c')),  c_enb = 1;  end
if ~isempty(strfind(opt.types, 'd')),  d_enb = 1;  end
if ~isempty(strfind(opt.types, 'i')),  i_enb = 1;  end
if ~isempty(strfind(opt.types, 'b')),  b_enb = 1;  end

if opt.plot_enb && b_enb
    c_enb = 1;  d_enb = 1;
end

PrintMsg('peek');

% images sizes
d_width = 512; d_height = 424; outOfRange = 5000;
c_width = 1920; c_height = 1080;

% % Create matrices for the images
% if d_enb,  camData.depth = zeros(d_height,d_width,'uint16');  end
% if i_enb,  camData.infrared = zeros(d_height,d_width,'uint16');  end
% if c_enb,  camData.color = zeros(c_height*opt.colorScale,c_width*opt.colorScale,3,'uint8');  end

n = 0;  validData = 0;  nMax = 100;  pauseTime = 0.02;
while n <= nMax && ~validData
    % Get frames from Kinect and save them on underlying buffer
    pause(pauseTime);
    validData = k2.updateKin2;
    n = n + 1;
end

if ~validData
    error('Kinect data update failed!');
end
PrintMsg('kinect updated', num2str(n*pauseTime));

camData.time = char(datetime('now','Format','yyyyMMdd_HHmmssMS'));

% Copy data to Matlab matrices
if d_enb,  
    camData.depth = k2.getDepth;  
    camData.depth(camData.depth>outOfRange) = outOfRange; % truncate depth
end
if c_enb,  camData.color = k2.getColor;  end
if i_enb,  camData.infrared = k2.getInfrared;  end
if d_enb && c_enb && opt.register
    
    [d_x, d_y] = meshgrid(1:d_width, 1:d_height);
    [c_x, c_y] = meshgrid(1:c_width, 1:c_height);
    % Using the mapping, map the points from depth coordinates to color coordinates
    % Input and output: n x 2 matrix (n points)
    % matColorPoints2Depth fails due to large size of c_x, c_y
    colorCoords = k2.mapDepthPoints2Color([d_x(:), d_y(:)]);
    idx_valid = find(colorCoords(:,1) > 0 & colorCoords(:,2) > 0 & colorCoords(:,1) <= c_width & colorCoords(:,2) <= c_height);
    camData.depth_hd_rect = zeros(c_height,c_width,'uint16');
    
    tic;
    for ii = 1:length(idx_valid)
        camData.depth_hd_rect(colorCoords(idx_valid(ii),2), colorCoords(idx_valid(ii),1)) = camData.depth(d_y(idx_valid(ii)), d_x(idx_valid(ii)));
    end
    tElapsed = toc;
    PrintMsg('DepthRect', num2str(tElapsed));
    
%     depth_hd_rect = zeros(length(idx_valid),1);
%     for ii = 1:length(idx_valid)
%         depth_hd_rect(ii) = double(camData.depth(d_y(idx_valid(ii)), d_x(idx_valid(ii))));
%     end
%     tic;
%     warning('off', 'MATLAB:scatteredInterpolant:DupPtsAvValuesWarnId')
%     camData.depth_hd_rect = uint16(griddata(double(colorCoords(idx_valid, 2)), double(colorCoords(idx_valid, 1)), depth_hd_rect, c_y, c_x));
%     tElapsed = toc;
%     PrintMsg('DepthRect', num2str(tElapsed));
end

% Get 3D bodies joints
% getBodies returns a structure array.
% The structure array (bodies) contains 6 bodies at most
% Each body has:
% -Position: 3x25 matrix containing the x,y,z of the 25 joints in
%   camera space coordinates
% -TrackingState: state of each joint. These can be:
%   NotTracked=0, Inferred=1, or Tracked=2
% -LeftHandState: state of the left hand
% -RightHandState: state of the right hand
if b_enb,  camData.bodies = k2.getBodies;  end

if opt.savedata,  save([opt.datapath, camData.time, '_camData.mat'], 'camData');  end

if opt.plot_enb
    if d_enb
        % depth stream figure
        d.h = figure;
        d.ax = axes;         

        % update depth figure
        depth8u = uint8(camData.depth*(255/outOfRange));
        depth8uc3 = repmat(depth8u,[1 1 3]);
        d.im = imshow(depth8uc3, 'Parent', d.ax);    
        % set(d.im,'CData',camData.depth);

        title('Depth Source (press q to exit)')
        if opt.savefig,  imwrite(depth8u, [opt.datapath, camData.time, '_depth.png']);  end

        if opt.register
            % depth stream figure
            d_hd.h = figure;
            d_hd.ax = axes;
            
            % update depth figure
            depth8u = uint8(camData.depth_hd_rect*(255/outOfRange));
            depth8uc3 = repmat(depth8u,[1 1 3]);
            depth8uc3 = imresize(depth8uc3,opt.colorScale);
            d_hd.im = imshow(depth8uc3, 'Parent', d_hd.ax);
            % set(d.im,'CData',camData.depth);
            
            title('Depth (HD rectified) Source (press q to exit)')
            if opt.savefig,  imwrite(depth8u, [opt.datapath, camData.time, '_depth_hd_rect.png']);  end
        end
    end
    
    if c_enb
        % color stream figure
        c.h = figure;
        c.ax = axes;

        % update color figure
        color = imresize(camData.color,opt.colorScale);
        c.im = imshow(color, 'Parent', c.ax);
        % set(c.im,'CData',camData.color);
        
        if opt.CalibrationFlip
            color = flip(color ,2); 
        end

        title('Color Source (press q to exit)');
        if opt.savefig,  imwrite(color, [opt.datapath, camData.time, '_color.png']);  end
        
        if d_enb
            figure
            imshowpair(depth8uc3, color)
            title('Color vs. Depth_hd_rect', 'FontSize', 14, 'Interpreter','none');
        end
    end
    
    if i_enb
        % color stream figure
        i.h = figure;
        i.ax = axes;
        
        % update infrared figure
        i.im = imshow(camData.infrared, 'Parent', i.ax);        
        %set(i.im,'CData',camData.infrared);

        title('Infrared Source (press q to exit)');
        if opt.savefig,  imwrite(camData.infrared, [opt.datapath, camData.time, '_infrared.png']);  end
    end   
    
    if b_enb
        % Number of bodies detected
        numBodies = size(camData.bodies,2);
        PrintMsg('body detected', num2str(numBodies));
        
        % Draw bodies on depth and color images
        % Parameters:
        % 1) image axes
        % 2) bodies structure
        % 3) Destination image (depth or color)
        % 4) Joints' size (circle raddii)
        % 5) Bones' Thickness
        % 6) Hands' Size
        k2.drawBodies(d.ax,camData.bodies,'depth',5,3,15);
        k2.drawBodies(c.ax,camData.bodies,'color',10,6,30);
        if opt.savefig,  saveas(d.h, [opt.datapath, camData.time, '_body_d.png']);  end
        if opt.savefig,  saveas(c.h, [opt.datapath, camData.time, '_body_c.png']);  end        
    end
    
%     closefigs(pauseTime);
    
end

end