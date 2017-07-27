% Wenjie Chen, FANUC Corporation, 2016/01/15
% Normalizes point sets to have zero mean and unit variance
% Modified from CPD_NORMALIZE Normalizes reference and template point sets to have zero

function  [X, normal] =pts_normalize(x, opt)

if nargin < 2,  opt = 'mean';  end

n = size(x,1);
if strcmpi(opt, 'mean')
    normal.xd = mean(x);
    x = x - repmat(normal.xd, n, 1);
    normal.xscale = sqrt(sum(sum(x.^2,2)) / n);
    X = x / normal.xscale;
else  % normalize on longer dimension
    normal.xd = (max(x) + min(x))/2;
    x = x - repmat(normal.xd, n, 1);
    normal.xscale = max(max(x) - min(x));
    x = x / normal.xscale;
    X = x + 0.5;
end
