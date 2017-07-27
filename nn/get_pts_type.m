% Wenjie Chen, FANUC Corporation, 2016/02/01
% Get point cloud data and corresponding types in matrix form X and T from
% all the mat files in a specified folder

function [X, T, Type_set] = get_pts_type(datapath, srctype, imgsize)

Type_set = {'doublefold', 'fold', 'line', 'tie'}; % 

if strcmpi(srctype, 'k2data')
    data_files = rdir([datapath, '*\**\*k2data.mat']);
    fNum = length(data_files);
    X = [];  T = [];
    
    for i = 1:fNum
        load(data_files(i).name, 'ptCld_ds', 'type_name');
        X_nrm = pts_normalize(ptCld_ds.Location);
        type_idx = strcmpi(type_name, Type_set);
        if sum(type_idx) == 1
            X(:, end+1) = X_nrm(:);
            T(:, end+1) = type_idx;
        end
    end
elseif strcmpi(srctype, 'pts') % pts.mat type
    data_files = rdir([datapath, '*\**\*pts.mat']);
    fNum = length(data_files);
    X = [];  T = [];
    
    for i = 1:fNum
        load(data_files(i).name, 'pts');
        ds_num = length(pts);
        for j = 1:ds_num
            type_idx = strcmpi(pts(j).type, Type_set);
            if sum(type_idx) == 1
                X(:, end+1) = pts(j).xyz(:);
                T(:, end+1) = type_idx;
            end
        end
    end
elseif strcmpi(srctype, 'k2dataimg') % 'img' type
    data_files = rdir([datapath, '*\**\*k2data.mat']);
    fNum = length(data_files);
    if nargin < 3,  imgsize = 28;  end
    X = {};  T = [];
    
    for i = 1:fNum
        load(data_files(i).name, 'ptCld', 'type_name');
        X2d = proj3Dto2D(pcdownsample(ptCld, 'random', 0.15));  
        type_idx = strcmpi(type_name, Type_set);
        if sum(type_idx) == 1
            X{end+1} = pts2img(X2d, imgsize);
            T(:, end+1) = type_idx;
        end
    end
elseif strcmpi(srctype, 'ptsimg') % 'img' type
    data_files = rdir([datapath, '*\**\*pts.mat']);
    fNum = length(data_files);
    X = {};  T = [];
    
    for i = 1:fNum
        load(data_files(i).name, 'pts');
        ds_num = length(pts);
        for j = 1:ds_num
            type_idx = strcmpi(pts(j).type, Type_set);
            if sum(type_idx) == 1
                X{end+1} = pts(j).img;
                T(:, end+1) = type_idx;
            end
        end
    end    
end

end