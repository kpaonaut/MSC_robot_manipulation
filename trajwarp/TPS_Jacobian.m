%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       get numerical Jacobian matrix of TPS function at given point 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Jacobian = TPS_Jacobian(func, x, eps)

% finite differences gradient calculation
dim = numel(x);
Jacobian = zeros(dim, dim);

for i = 1:dim
    dx = zeros(dim, 1);
    dx(i) = eps;
    Jacobian(:, i) = (func(x + dx) - func(x - dx)) / (2 * eps);
end

end
