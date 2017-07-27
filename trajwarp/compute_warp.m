%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       get TPS function parameters from old/new feature points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, B, c] = compute_warp(S, T, varargin)

% S are old feature points (in columns)
% T are cooresponding new points we would like to match to (in columns)

lambda = 1;
if ~isempty(varargin)
    lambda = varargin{1};
end
[d,n] = size(S);

% Build Kernel Matrix K for three dimensional TPS
K = zeros(n,n);    % each j column is s_j
for j = 1:n
    for i = 1:n
        dis = norm(S(:,j)-S(:,i) ,2);
        if dis == 0
            K(i,j) = 0;
        else
            if d == 2
                K(i,j) = dis^2*log(dis);
            elseif d == 3
                K(i,j) = -dis;
            end
        end
    end
end

% use CVX to solve the optimized TPS function (A,B,c) 
sqrt_K = sqrtm(K);
cvx_begin
variables A(n,d)  B(d,d)  c(d,1) ;
minimize(square_pos(norm(T-A'*K-B*S-c*ones(n,1)','fro')) + lambda*square_pos(norm(A'*sqrt_K,'fro')))         
subject to
S*A == zeros(d,d);
ones(n,1)'*A == zeros(1,d);
cvx_end

end
