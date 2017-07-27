%% Wenjie Chen, 2016/07/12, FANUC Corporation
% Convert transformation matrix to xyzwpr

function xyzwpr = T2xyzwpr(T, opt)

if nargin < 2,  opt = [];  end
if ~isfield(opt,'SI_tcp') || isempty(opt.SI_tcp),  opt.SI_tcp = 0;  end   % 0 - degree, mm (default); 1 - radian, m

if size(T, 1) ~= 4,  T = permute(T, [2, 3, 1]);  end
if size(T, 2) ~= 4,  T = permute(T, [1, 3, 2]);  end

xyzwpr = zeros(size(T, 3), 6);
xyzwpr(:,1:3) = transl(T) * 1000;           % unit - mm
xyzwpr(:,6:-1:4) = tr2rpy(T, 'deg', 'zyx'); % unit - deg

if opt.SI_tcp,  xyzwpr(:,4:6) = deg2rad(xyzwpr(:,4:6));  xyzwpr(:,1:3) = xyzwpr(:,1:3)/1000;  end;  % make xyzwpr - m, rad

end
