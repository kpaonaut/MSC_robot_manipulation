% Wenjie Chen, FANUC Corporation, 2016/03/15
% Combine neural network imdb data for MatConvNet
% Inputs:   imdbSet     - imdb data sets for combining
%           rmMean      - 1 for remove mean data, 0 for not removing mean

function imdb = combineNNimdb(imdbSet, rmMean)

set = [];   data = [];   labels = [];
metaSets = imdbSet{1}.meta.sets;
metaClasses = imdbSet{1}.meta.classes;

if nargin < 2
    rmMean = 0;
end

for i = 1:numel(imdbSet)
    set = cat(2, set, imdbSet{i}.images.set);
    data = cat(4, data, imdbSet{i}.images.data);
    labels = cat(2, labels, imdbSet{i}.images.labels);
    if (~isempty(setdiff(metaSets, imdbSet{i}.meta.sets)) || ...
            ~isempty(setdiff(imdbSet{i}.meta.sets, metaSets)) || ...
            ~isempty(setdiff(metaClasses, imdbSet{i}.meta.classes)) || ...
            ~isempty(setdiff(imdbSet{i}.meta.classes, metaClasses)))
        error('IMDB sets have different set names or classes!');
    end
end

if rmMean
    dataMean = mean(data(:,:,:,set == 1), 4);
else
    dataMean = single(zeros(size(squeeze(data(:,:,1,1)))));
end
data = bsxfun(@minus, data, dataMean) ;

imdb.images.data = data ;
imdb.images.data_mean = dataMean;
imdb.images.labels = labels;
imdb.images.set = set;
imdb.meta.sets = metaSets;
imdb.meta.classes = metaClasses;

end