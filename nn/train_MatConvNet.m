% Wenjie Chen, FANUC Corporation, 2016/02/24

if exist('../init_setup.m', 'file')
    run('../init_setup.m');
end

%% Get training data and test data
disp('Loading training data and test data ...');

imSize = 28;
train_set = 'train_NN_2d_1000_28_W02';
train_path = ['..\data\', train_set, '\'];
test_path = '..\data\test\';

expDir = ['..\data\MatConvNet\', train_set, '\'];
imdbPath = fullfile(expDir, 'imdb.mat');
if ~exist(imdbPath, 'file')
    imdb_train = getNNimdb(train_path, 'ptsimg', imSize, 'train', 0);
    imdb_test = getNNimdb(test_path, 'k2dataimg', imSize, 'test', 0);
    imdb = combineNNimdb({imdb_train, imdb_test}, 1);
    if isempty(ls(expDir)),  mkdir(expDir);  end
    save(imdbPath, '-struct', 'imdb') ;
else
    imdb = load(imdbPath);
end

%% Experiment with the cnn_mnist_fc_bnorm
disp('Constructing and training neural network classifier ...');
tStart = tic;
[net_bn, info_bn] = cnn_mnist(...
  'expDir', expDir, 'batchNormalization', true);
toc(tStart)

figure(1) ; clf ;
subplot(1,2,1) ;
semilogy(info_bn.val.objective', '+--') ;
xlabel('Training samples [x 10^3]'); ylabel('energy') ;
grid on ;
title('objective') ;
subplot(1,2,2) ;
plot(info_bn.val.error', '+--') ;
h=legend('BNORM-val','BNORM-val-5') ;
grid on ;
xlabel('Training samples [x 10^3]'); ylabel('error') ;
set(h,'color','none') ;
title('error') ;
drawnow ;

%% Test results
if exist('../init_setup.m', 'file')
    run('../init_setup.m');
end

% load trained net
train_set = 'train_NN_2d_1000_28_W02';
expDir = ['..\data\MatConvNet\', train_set, '\'];
load([expDir, 'net-epoch-20.mat']);
imdb = load([expDir, 'imdb.mat']);
net = cnn_imagenet_deploy(net);
vl_simplenn_display(net);

% load test data
imSize = 28;
test_path = '..\data\test\';
imdb_test = getNNimdb(test_path, 'k2dataimg', imSize, 'test', 0);
imdb_test.images.data_mean = imdb.images.data_mean;
imdb_test.images.data = bsxfun(@minus, imdb_test.images.data, imdb_test.images.data_mean) ;

% test result
res = vl_simplenn(net, imdb_test.images.data, [], [], 'mode', 'test');
scores = squeeze(gather(res(end).x));
[bestScore, best] = max(scores);
disp(['Actual type:     ', num2str(imdb_test.images.labels)]);
disp(['Identified type: ', num2str(best)]);
wrong_idx = find(imdb_test.images.labels - best)
imdb_test.images.labels(wrong_idx)
best(wrong_idx)
% bestScore(wrong_idx)
% idx = [1:11, 26:36, 51:61, 76:86];
% sum(ismember(wrong_idx, idx))
