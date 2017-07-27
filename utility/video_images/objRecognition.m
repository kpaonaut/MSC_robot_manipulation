% Wenjie Chen, FANUC Corporation, 2016/04/19
% Based on VIDEOPATTERNMATCHING, video pattern matching demo from MATLAB

%% Initial setup (path);
if exist('../init_setup.m', 'file')
    run('../init_setup.m');
end

%% Initialization
% Initialize required variables such as the threshold value for the cross
% correlation and the decomposition level for Gaussian Pyramid
% decomposition.
threshold = single(0.99);
level = 1;
imgPath = 'E:\Chin\Work\20151126_Skill_Learning\20151218_State_Mapping\data\picture\';
imgFile = [imgPath, '20160420_10592643_color.png'];
Im = imread(imgFile);
Im = rgb2gray(Im);
Im = im2single(Im);

%%
obj_img_File = [imgPath, '20160419_19062042_color', '*.png'];
obj_img_list = ls(obj_img_File);
obj_img_list = obj_img_list(2:end, :);
numberOfTargets = 1;

%%
% Create three gaussian pyramid System objects for decomposing the target
% template and decomposing the Image under Test(IUT). The decomposition is
% done so that the cross correlation can be computed over a small region
% instead of the entire original size of the image.
hGaussPymd1 = vision.Pyramid('PyramidLevel',level);
hGaussPymd2 = vision.Pyramid('PyramidLevel',level);

%%
% Create a System object to rotate the image by angle of pi before
% computing multiplication with the target in the frequency domain which is
% equivalent to correlation.
hRotate1 = vision.GeometricRotator('Angle', pi);

%%
% Create two 2-D FFT System objects one for the image under test and the
% other for the target.
hFFT2D1 = vision.FFT;
hFFT2D2 = vision.FFT;

%%
% Create a System object to perform 2-D inverse FFT after performing
% correlation (equivalent to multiplication) in the frequency domain.
hIFFFT2D = vision.IFFT;

%%
% Create 2-D convolution System object to average the image energy in tiles
% of the same dimension of the target.
hConv2D = vision.Convolver('OutputSize','Valid');

%%
% Create a System object to display the tracking of the pattern.
sz = get(0,'ScreenSize');
pos = [100 100 sz(3)*0.8 sz(4)*0.8];
hROIPattern = vision.VideoPlayer('Name', 'Overlay the ROI on the target', ...
    'Position', pos);

%%
Im_gp = step(hGaussPymd2, Im);
[ri, ci]= size(Im_gp);
IUT_energy = (Im_gp).^2;

for i = 1:size(obj_img_list,1)
    target_image = imread([imgPath, obj_img_list(i,:)]);
    target_image = rgb2gray(target_image);
    target_image = imrotate(target_image, 50, 'crop');
    
    % Here you implement the following sequence of operations.
    
    % Downsample the target image by a predefined factor using the
    % gaussian pyramid System object. You do this to reduce the amount of
    % computation for cross correlation.
    target_image = single(target_image);
    target_dim_nopyramid = size(target_image);
    target_image_gp = step(hGaussPymd1, target_image);
    target_energy = sqrt(sum(target_image_gp(:).^2));
    
    % Rotate the target image by 180 degrees, and perform zero padding so that
    % the dimensions of both the target and the input image are the same.
    target_image_rot = step(hRotate1, target_image_gp);
    [rt, ct] = size(target_image_rot);
    r_mod = 2^nextpow2(rt + ri);
    c_mod = 2^nextpow2(ct + ci);
    target_image_p = [target_image_rot zeros(rt, c_mod-ct)];
    target_image_p = [target_image_p; zeros(r_mod-rt, c_mod)];
    
    % Compute the 2-D FFT of the target image
    target_fft = step(hFFT2D1, target_image_p);
    
    % Initialize constant variables used in the processing loop.
    target_size = repmat(target_dim_nopyramid, [numberOfTargets, 1]);
    gain = 2^(level);
    Im_p = zeros(r_mod, c_mod, 'single'); % Used for zero padding
    C_ones = ones(rt, ct, 'single');      % Used to calculate mean using conv
    
    %%
    % Create a System object to calculate the local maximum value for the
    % normalized cross correlation.
    hFindMax = vision.LocalMaximaFinder( ...
        'Threshold', single(-1), ...
        'MaximumNumLocalMaxima', numberOfTargets, ...
        'NeighborhoodSize', floor(size(target_image_gp)/2)*2 - 1);
    
    %% Video Processing Loop
    % Create a processing loop to perform pattern matching on the input video.
    % This loop uses the System objects you instantiated above. The loop is
    % stopped when you reach the end of the input file, which is detected by
    % the |VideoFileReader| System object.
    
    % Frequency domain convolution.
    Im_p(1:ri, 1:ci) = Im_gp;    % Zero-pad
    img_fft = step(hFFT2D2, Im_p);
    corr_freq = img_fft .* target_fft;
    corrOutput_f = step(hIFFFT2D, corr_freq);
    corrOutput_f = corrOutput_f(rt:ri, ct:ci);
    
    % Calculate image energies and block run tiles that are size of
    % target template.
    IUT = step(hConv2D, IUT_energy, C_ones);
    IUT = sqrt(IUT);
    
    % Calculate normalized cross correlation.
    norm_Corr_f = (corrOutput_f) ./ (IUT * target_energy);
    xyLocation = step(hFindMax, norm_Corr_f);
    
    % Calculate linear indices.
    linear_index = sub2ind([ri-rt, ci-ct]+1, xyLocation(:,2),...
        xyLocation(:,1));
    
    norm_Corr_f_linear = norm_Corr_f(:);
    norm_Corr_value = norm_Corr_f_linear(linear_index);
    detect = (norm_Corr_value > threshold);
    target_roi = zeros(length(detect), 4);
    ul_corner = (gain.*(xyLocation(detect, :)-1))+1;
    target_roi(detect, :) = [ul_corner, fliplr(target_size(detect, :))];
    
    % Draw bounding box.
    Im = insertShape(Im, 'Rectangle', target_roi, 'Color', 'green');
    
end

step(hROIPattern, Im);
