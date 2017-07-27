%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       function to warp transformation matrix 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function T_warped = TPS_warp_T(T, warp)

translation = T(1:3,4);
rotation = T(1:3,1:3);

% warp tranlation part
translation_warped = warp(translation);

% warp rotation part
eps = 1e-3;
Jacobian = TPS_Jacobian(warp, translation, eps);
rotation_warped = Jacobian * rotation;
% orthgonalize rotation matrix
[U,~,V] = svd(rotation_warped);
rotation_warped = U*V';

T_warped = [[rotation_warped, translation_warped]; [0,0,0,1]];

end
