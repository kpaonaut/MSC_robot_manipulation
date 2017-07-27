%  Nonrigid Example 6. Coherent Point Drift (CPD).
%  Nonrigid registration of 3D bunny point sets with use of FGT and Lowrank kernel approximation.

% Wenjie Chen, FANUC Corporation, 2016/01/24

if exist('../init_setup.m', 'file')
    run('../init_setup.m');
end
plot_enb = 0;

%% Init full set of options %%%%%%%%%%
opt.method='nonrigid_lowrank'; % use nonrigid registration with lowrank kernel approximation
opt.numeig=30;                 % leave only 30 larges (out of 8171) eigenvectors/values to approximate G
opt.eigfgt=1;                  % use FGT to find the largest eigenvectore/values 

opt.beta=1;             % the width of Gaussian kernel (smoothness)
opt.lambda=3;           % regularization weight

opt.viz=0;              % show every iteration
opt.outliers=0.1;       % use 0.7 noise weight
opt.fgt=2;              % use FGT to compute matrix-vector products (2 means to switch to truncated version at the end, see cpd_register)
opt.normalize=1;        % normalize to unit variance and zero mean before registering (default)
opt.corresp=1;          % compute correspondence vector at the end of registration (not being estimated by default)

opt.max_it=100;         % max number of iterations
opt.tol=1e-3;           % tolerance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Register two sets of data
test_path = '.\data\test';
test_files = rdir([test_path, '\**\*.mat']);
fNum_X = length(test_files);
X = cell(fNum_X, 1);  XType = cell(fNum_X, 1);
for i = 1:fNum_X
    load(test_files(i).name, 'ptCld_ds', 'type_name');
    X{i} = ptCld_ds.Location;
    XType{i} = type_name;
end

train_path = '.\data\train';
train_files = rdir([train_path, '\**\*.mat']);
fNum_Y = length(train_files);
Y = cell(fNum_Y, 1);  YType = cell(fNum_Y, 1);
for i = 1:fNum_Y
    load(train_files(i).name, 'ptCld_ds', 'type_name');
    Y{i} = ptCld_ds.Location;
    YType{i} = type_name;
end

XType_est = cell(fNum_X, 1);
for xi = 1:fNum_X

    Transform = cell(fNum_Y, 1);  C = cell(fNum_Y, 1);  L_set = zeros(fNum_Y, 1);
    for i = 1:fNum_Y
        disp(['Now registrating to scene ', num2str(i)]);
        [Transform{i}, C{i}]=cpd_register(X{xi}, Y{i}, opt);
        L_set(i) = Transform{i}.L;
    end
    [L_sort, L_idx] = sort(L_set);
%     disp('Final sorted likelihood :');
%     disp(num2str([L_sort, L_idx]));
    XType_est{xi} = YType{L_idx(1)};
    disp(['Test No. ', num2str(xi), ':  Actual Type - ', upper(XType{xi}), ',  Estimated Type - ', upper(XType_est{xi})]);
    
    %% Plot the final results
    if plot_enb
        scrsz = get(groot,'ScreenSize');
        fh = figure('Position',[scrsz(3)*0.1 scrsz(4)*0.1 scrsz(3)*0.8 scrsz(4)*0.8]);
        for i = 1:3
            subplot(2,3,i),cpd_plot_iter(X{xi}, Y{L_idx(i)}, Transform{L_idx(i)}.C); grid on; view(45,60); title('Before');
            subplot(2,3,i+3),cpd_plot_iter(X{xi}, Transform{L_idx(i)}.Y); grid on; view(45,60); title('After registering Y to X');
        end
        closefigs;
    end
    
end

disp(strcmpi(XType, XType_est));
