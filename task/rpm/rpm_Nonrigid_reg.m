% TPS-RPM seems not very stable. Each time may generate a different result.
% cMIX_dual (following rraprentice) is even worse than cMIX

% Wenjie Chen, FANUC Corporation, 2016/01/24

if exist('../init_setup.m', 'file')
    run('../init_setup.m');
end
plot_enb = 0;

%% Init full set of options %%%%%%%%%%
frac       = 3;
T_init     = 0.5;
T_finalfac = 500;
disp_flag  = 0;
m_method   = 'mixture';

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
    
    Transform = cell(fNum_Y, 1);  L_set = zeros(fNum_Y, 1);
    for i = 1:fNum_Y
        [X_nrm, Y_nrm, normal] = cpd_normalize(X{xi},Y{i});
        
        disp(['Now registrating to scene ', num2str(i)]);
        [c, d, m, vy, E] = cMIX(Y_nrm, X_nrm, frac, T_init, T_finalfac, disp_flag, m_method);
        %     [c, d, m, vy, E] = cMIX_dual(Y_nrm, X_nrm, frac, T_init, T_finalfac, disp_flag, m_method);
        
        [N, D] = size(X_nrm);  [M, D] = size(Y_nrm);
        sigma2 = (M*trace(X_nrm'*X_nrm) + N*trace(Y_nrm'*Y_nrm) - 2*sum(X_nrm)*sum(Y_nrm)') / (M*N*D);
        if sigma2 < 0.05, sigma2 = 0.05; end;
        % Find the correspondence, such that Y corresponds to X(C,:)
        Transform{i}.C = cpd_Pcorrespondence(X_nrm, vy, sigma2, 0.1);  % outliers = 0.1, generated C is not accurate
        Transform{i}.M = m;
        
        Transform{i}.method = m_method;
        Transform{i}.c = c;
        Transform{i}.d = d;
        %     Transform{i}.Y = vy;
        Transform{i}.Y = vy * normal.xscale + repmat(normal.xd, size(vy,1), 1);
        Transform{i}.L = E;
        Transform{i}.normal = normal;
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

