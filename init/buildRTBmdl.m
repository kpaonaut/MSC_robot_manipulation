%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%          Build Robot Model Using Robotic Toolbox           %
%                   Wenjie Chen, 2016/06/24                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function rtb = buildRTBmdl(r)

% define each link
for i = 1:r.n
    L(i) = Revolute('d',r.DH(2,i),'a',r.DH(3,i),'alpha',r.DH(4,i),'offset',r.DH(1,i));
end

% construct full robot 
rtb = SerialLink(L, 'name', ['Robot No. ', num2str(r.robot_no)]);
rtb.tool = [eye(3), [0 0 r.DH(2,r.n+1)]'; 0, 0, 0, 1];
rtb.qlim = pi/180*[r.rj_pos_limit_min; r.rj_pos_limit_max]';

fprintf(['---> RTB model for ', rtb.name, ' constructed!\n']);

end
