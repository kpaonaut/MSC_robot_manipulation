function reg_2imgs(fname_fixed, fname_moving, opt)
% Input arguments:
%       fname_fixed      - filename of fixed image
%       fname_moving     - filename of moving image
%       opt              - 'gpu': use gpu processing; 
%                          others: do not use gpu (default)
%
% Following the example of MATLAB function 'imregdemons'.
%
% Wenjie Chen, FANUC Corporation, 2015/12/21

if nargin < 3,  opt = 'normal';  end

%% imregdemons
fixed  = imread(fname_fixed);
moving = imread(fname_moving);
% figure
% imshowpair(fixed,moving,'montage')
figure
imshowpair(fixed,moving)
title('Fixed vs. Moving', 'FontSize', 14);

if ~strcmp(opt, 'gpu')
    fixed  = rgb2gray(fixed);
    moving = rgb2gray(moving);
    
    moving = imhistmatch(moving,fixed);
    [~,movingReg] = imregdemons(moving,fixed,[500 400 200],...
        'AccumulatedFieldSmoothing',1.3);
else
    fixedGPU  = gpuArray(fixed);
    movingGPU = gpuArray(moving);
    fixedGPU  = rgb2gray(fixedGPU);
    movingGPU = rgb2gray(movingGPU);
    
    fixedHist = imhist(fixedGPU);
    movingGPU = histeq(movingGPU,fixedHist);
    [~,movingReg] = imregdemons(movingGPU,fixedGPU,[500 400 200],...
        'AccumulatedFieldSmoothing',1.3);
    movingReg = gather(movingReg);
end

figure
imshowpair(fixed,movingReg)
title('Fixed vs. MovingReg', 'FontSize', 14);
figure
imshowpair(moving,movingReg)
title('Moving vs. MovingReg', 'FontSize', 14);
% figure
% imshowpair(fixed,movingReg,'montage')

end