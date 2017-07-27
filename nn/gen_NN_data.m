% Wenjie Chen, FANUC Corporation, 2016/01/24

if exist('../init_setup.m', 'file')
    run('../init_setup.m');
end

dim_NN_pts = 2;
imgsize = 50;
save_enb = 1;
plot_enb = 0;
trans_type = 'cpd'; % 'tps'
rand_type = 'uniform'; % 'normal'
t_num = 10000;

%%
Type_set = {'doublefold', 'fold', 'line', 'tie'};
train_path = ['..\data\train_NN_', num2str(dim_NN_pts), 'd\'];
train_path0 = '..\data\train\';

if strcmpi(trans_type, 'cpd')
    beta=1;             % the width of Gaussian kernel (smoothness)
    numeig=30;          % leave only 30 larges eigenvectors/values to approximate G
    eigfgt=1;           % use FGT to find the largest eigenvectore/values
end
                
for j = 1:length(Type_set)
    disp(['============== Generate Data for ', upper(Type_set{j}), ' Type ==============']);
    
    %% Load original data
    data_files = rdir([train_path0, '\', Type_set{j}, '\*.mat']);
    if save_enb
        filepath = [train_path, Type_set{j}, '\'];
        if isempty(ls(filepath)),  mkdir(filepath);  end
    end
    
    for fi = 1:length(data_files)
        if dim_NN_pts == 3
            load(data_files(fi).name, 'ptCld_ds', 'type_name');
            X = ptCld_ds.Location;
        else
            load(data_files(fi).name, 'ptCld', 'type_name');  % Not using downsampled point clouds
            X = proj3Dto2D(pcdownsample(ptCld,'random',0.15));  
        end
        [N, D] = size(X);
        
        %% Transformation
        [X_nrm, normal] = pts_normalize(X);
        if strcmpi(trans_type, 'cpd')
            [Q, S] = cpd_GRBF_lowrankQS(X_nrm, beta, numeig, eigfgt);
            d = eye(D, D);       % not used in CPD
        else    % 'tps'
            Z = X_nrm;
        end        
    
        if plot_enb
            scrsz = get(groot,'ScreenSize');
            fh = figure('Position',[scrsz(3)*0.1 scrsz(4)*0.1 scrsz(3)*0.8 scrsz(4)*0.8]);
        end
        
        pts = struct;
        for i = 1:t_num
            if i == 1
                VX = X_nrm;     % use normalized point clouds for training
                normal_vx.xd = zeros(1,D);  normal_vx.xscale = 1;
                if strcmpi(trans_type, 'cpd')
                    W = zeros(N,D);     
                else    % 'tps'
                    W = zeros(N,D+1);
                    d = eye(D+1,D+1);
                end
            else
                if strcmpi(trans_type, 'cpd')           % CPD: pts1 = pts + G*W; pts1 = pts1 * R;
                    if strcmpi(rand_type, 'uniform')
                        W = (rand(N,D) - 0.5)*0.2;      % smaller c_tps makes warp pts smoother
                        d = rand(1,D) * 2 * pi;         % rotation angle
                    else % 'normal'
                        W = randn(N,D) * 0.05;
                        d = randn(1,D) / 5 * 2 * pi;    % rotation angle
                    end
                    if dim_NN_pts == 2,  W = W * 0.2;  R = rotz(d(1));  R = R(1:2, 1:2);  end
                    if dim_NN_pts == 3,  R = rpy2r(d);  end
                    VX = X_nrm + Q*(S*(Q'*W));
                    VX = VX * R;                    % doing rotations
                else                                % TPS: pts1 = pts*d + G*W;
                    if strcmpi(rand_type, 'uniform')
                        W = (rand(N,D+1) - 0.5)*0.02;   % smaller c_tps makes warp pts smoother
                        d = rand(D+1,D+1) - 0.5;
                    else % 'normal'
                        W = randn(N,D+1) * 0.005;   % smaller c_tps makes warp pts smoother
                        d = randn(D+1,D+1);
                    end                        
                    if dim_NN_pts == 2,  W = W * 0.05;  end
%                     W(:,1) = W(:,1) * 1e-13;         % Not necessary since the first column is disgarded during warping          
                    [VX] = cMIX_warp_pts (trans_type, X_nrm, Z, W, d);
%                     W = W / normal_vx.xscale;
%                     d(1, 2:D+1) = d(1, 2:D+1) - normal_vx.xd;
%                     d = d / normal_vx.xscale;
                end
                [VX, normal_vx] = pts_normalize(VX);    % use normalized point clouds for training
            end

            if plot_enb
                subplot(2, t_num/2, i); cpd_plot_iter(X_nrm, VX); grid on; title('Orignal vs. Transformed'); axis equal;
                if dim_NN_pts == 3,  view(45,60);   end
%                 [VX1] = cMIX_warp_pts (trans_type, X_nrm, Z, W, d);  % 'tps'
%                 VX1 = X_nrm + Q*(S*(Q'*W));  % 'cpd'
%                 VX1 = (VX1 - repmat(normal_vx.xd, N, 1)) / normal_vx.xscale;  % normalization, This should be the same as VX_nrm above
%                 subplot(2, t_num/2, i); cpd_plot_iter(VX, VX1, (1:N)); grid on; view(45,60); title('Orignal vs. Transformed'); axis equal  % the correspondance is the original order
            end
            
            pts(i).xyz = VX;
            if dim_NN_pts == 2,  pts(i).img = pts2img(VX, imgsize);  end
            pts(i).W = W;
            pts(i).d = d;
            pts(i).type = type_name;
            pts(i).trans_type = trans_type;
            pts(i).rand_type = rand_type;
            pts(i).normal = normal;
            pts(i).normal_vx = normal_vx;
        end
        
        closefigs;
        
        if save_enb,  save([filepath, data_files(fi).name(end-27:end-10), 'pts.mat'], 'pts');  end
    end
end
