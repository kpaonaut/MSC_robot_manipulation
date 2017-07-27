% Wenjie Chen, FANUC Corporation, 2016/03/15
% Get neural network imdb data for MatConvNet
% Inputs:   dataPath    - path for data files
%           dataType    - 'k2dataimg' or 'ptsimg'
%           imSize      - image data size, e.g., 28
%           setType     - 'train', 'val', or 'test'
%           rmMean      - 1 for remove mean data, 0 for not removing mean

function imdb = getNNimdb(dataPath, dataType, imSize, setType, rmMean)
[xData, yData] = get_pts_type(dataPath, dataType, imSize);

x = zeros(imSize,imSize,numel(xData));
y = zeros(1,numel(xData));
for i = 1:numel(xData)
    x(:,:,i) = xData{i};
    y(i) = find(yData(:,i));
end

x = permute(reshape(x,imSize,imSize,numel(xData)),[2 1 3]) ;
y = double(y);

imdb.meta.sets = {'train', 'val', 'test'} ;
imdb.meta.classes = arrayfun(@(x)sprintf('%d',x),0:3,'uniformoutput',false) ;

set = ones(1,numel(y)) * find(strcmpi(setType, imdb.meta.sets));
data = single(reshape(x,imSize,imSize,1,[]));

if rmMean
    dataMean = mean(data(:,:,:,set == 1), 4);
else
    dataMean = single(zeros(size(squeeze(data(:,:,1,1)))));
end
data = bsxfun(@minus, data, dataMean) ;

imdb.images.data = data ;
imdb.images.data_mean = dataMean;
imdb.images.labels = y;
imdb.images.set = set;

end