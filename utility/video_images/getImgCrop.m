% Wenjie Chen, FANUC Corporation, 2016/04/19
% Get cropped image using GUI and save croped images
% Based on VIDEOPATTERN_GETTEMPLATE, Helper function used in videopatternmatching demo
% to get the template pattern to track.

function [img, numTargets, target_img] = getImgCrop(imgFile)

numTargets = 0;

% Read the image file
img = imread(imgFile);

% Pick some initial location for the target rectangle
roi = [size(img,2)/2, size(img,1)/2, 50, 50];
target_img = {};

hf = figure('Color', get(0, 'defaultuicontrolbackgroundcolor'), ...
    'Name', 'Target pattern', ...
    'NumberTitle', 'off');
imshow(img);

h = imrect(gca, roi);
api = iptgetapi(h);
api.setColor([0 1 0]);
api.addNewPositionCallback(@(p) title(mat2str(p)));

% Don't allow the rectangle to be dragged outside of image boundaries
fcn = makeConstrainToRectFcn('imrect',get(gca,'XLim'),get(gca,'YLim'));
api.setDragConstraintFcn(fcn);

yshift = 10;
uicontrol(hf, 'style', 'pushbutton', 'Units', 'Pixels', ...
    'String', 'Save', ...
    'Position', [size(img,2)/2 yshift 100 20], ...
    'Callback', @saveFcn);
uicontrol(hf, 'style', 'pushbutton', 'Units', 'Pixels', ...
    'String', 'Finish', ...
    'Position', [size(img,2)/2+110 yshift 100 20], ...
    'Callback', @finishFcn);
uiwait;

    function saveFcn(varargin)
        roi = api.getPosition();
        
        % Extract the template data
        target_img{end+1} = imcrop(img,roi);
        % assignin('base','target_img', target_img);
        sz_img = size(target_img{end});
        if any(sz_img(1:2) < 20) || any(sz_img(1:2) > 100)
            errordlg('Target height and width must be between 20 and 100 pixels.',...
                'Invalid dimensions');
            return;
        end
        
        numTargets = numTargets + 1;
        target_File = [imgFile(1:end-4), num2str(numTargets), imgFile(end-3:end)];
        imwrite(target_img{end}, target_File);
    end

    function finishFcn(varargin)
        close(hf);
    end
end


