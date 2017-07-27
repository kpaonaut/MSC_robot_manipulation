% Wenjie Chen, FANUC Corporation, 2016/03/23
% Transform cable data into format ready for classification

function cnnData = getData4Classfier(ptCld, imSize, data_mean)

if nargin < 2,  imSize = 28;  end
if nargin < 3,  data_mean = single(zeros(imSize, imSize));  end

x2D = proj3Dto2D(pcdownsample(ptCld, 'random', 0.15)); 
img2D = pts2img(x2D, imSize);
xData = permute(reshape(img2D,imSize,imSize,1),[2 1 3]) ;
xData = single(reshape(xData,imSize,imSize,1,[]));
cnnData = bsxfun(@minus, xData, data_mean) ;

end