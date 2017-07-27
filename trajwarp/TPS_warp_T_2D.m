%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       function to warp transformation matrix 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function T_warped = TPS_warp_T_2D(T, warp)

translation_xy = T(1:2,4);
translation_z = T(3,4);
rotation = T(1:3,1:3);

% warp tranlation part
translation_xy_warped = warp(translation_xy);

% warp rotation part
eps = 1e-4;
Jacobian_xy = TPS_Jacobian(warp, translation_xy, eps); %2-by-2

[U,~,V] = svd(Jacobian_xy);
Jacobian_xy = U*V';

Jacobian = [[Jacobian_xy, [0;0]]; [0,0,1]];


rotation_warped = Jacobian * rotation;
% orthgonalize rotation matrix
[U,~,V] = svd(rotation_warped);
rotation_warped = U*V';

T_warped = [[rotation_warped, [translation_xy_warped; translation_z]]; [0,0,0,1]];

end
