%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%  Transform coord from kinect frame to world frame
%  Specific data only applicable in MSC Lab's settings
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Rui Wang, 08/03/2017       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function points_W = transformU2W(points_U)
    T_W2U = transl((22.5 * 2.54 + 2.486 * 5) / 100, -2.486 * 2.5 / 100, 0) * trotz(-pi / 2); % transformation matrix, World to Kinect
    T_B2U = inv(si.ri{1}.T_B2W) * T_W2U; % transformation matrix, Robot Base to Kinect

    % transform points from kinect frame to world frame
    points_U = [points_U, ones(size(points_U, 1), 1)];
    points_W = (T_W2U * points_U')';
    points_W = points_W(:, 1:3);
end