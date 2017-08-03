%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%  Generate the orientation of the rope in test by TSM-RPM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Rui Wang, 08/02/2017       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function points_test_q = getOrientation(points_train, m)
% *** Paper equation (13)
points_train_q = getQ(points_train); % extract tangent space from cartesian space
points_test_q = m'(1 : size(m, 2)-1, 1 : size(m, 1)-1) * points_train_q;
n = size(points_test_q, 1);
x = 1 : n;
subplot(2, 1, 1); plot(x, points_train_q); title('train');
subplot(2, 1, 2); plot(x, points_test_q); title('test');
% m has additional rows and columns to deal with outliers
end