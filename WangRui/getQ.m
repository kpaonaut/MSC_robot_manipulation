%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Get the degree info of a rope
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Rui Wang, 08/02/2017       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  !Note!: Only an approximation, using CoM to calculate
%  instead of using real angle (which is hard to get)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = getQ(points)
q = zeros(size(points, 1), 1); % init q, allocate space
for i = 1 : size(q, 1) - 1
    dx = points(i + 1, 1) - points(i, 1);
    dy = points(i + 1, 2) - points(i, 2);
    q[i] = atan2(dy, dx) / pi * 180; % unit: degree, (-180, 180]
end
q(end) = q(end - 1);
end