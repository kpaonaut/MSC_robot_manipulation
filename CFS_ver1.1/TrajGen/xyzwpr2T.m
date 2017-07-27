%% Wenjie Chen, 2016/07/12, FANUC Corporation
% Convert xyzwpr to transformation matrix

function T = xyzwpr2T(xyzwpr, opt)

if nargin < 2,  opt = [];  end
if ~isfield(opt,'SI_tcp') || isempty(opt.SI_tcp),  opt.SI_tcp = 0;  end   % 0 - degree, mm (default); 1 - radian, m

if size(xyzwpr,2) ~= 6,  xyzwpr = xyzwpr';  end
num = size(xyzwpr,1);

if opt.SI_tcp,  xyzwpr(:,4:6) = rad2deg(xyzwpr(:,4:6));  xyzwpr(:,1:3) = xyzwpr(:,1:3)*1000;  end;  % make xyzwpr - mm, deg;
T = zeros(4,4,num);

for i = 1 : num 
    T(:,:,i) = [rpy2r(xyzwpr(i,6:-1:4), 'zyx', 'deg'), xyzwpr(i,1:3)'/1000; 0, 0, 0, 1];    % T - always using unit (m)
end

end
