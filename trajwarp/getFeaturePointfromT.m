%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Get Feature Points from Transformation Matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function featurePoints_xyz = getFeaturePointfromT(featurePoints_T, offset)

% featurePoints_T(4,4,N)
N = size(featurePoints_T,3);
% Each T generates 7 feature points (0, +x, -x, +y, -y, +z, -z)
direction = [ 0,0,0;...
    1,0,0;...
    0,1,0;...
    0,0,1;...
    -1,0,0;...
    0,-1,0;...
    0,0,-1];

featurePoints_xyz = zeros(3,7*N);
for n = 1:N
    T = featurePoints_T(:,:,n);
    for i = 1:7
        T_new = T * transl(offset*direction(i,:));
        featurePoints_xyz(:,7*(n-1)+i) = T_new(1:3,4);
    end
end
