%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       get warping function handle from TPS parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function warp = make_warp(A, B, c, data_array)
  warp = @(warp_pt) compute_warp(warp_pt, A, B, c, data_array);
end

function pt = compute_warp(warp_pt, A, B, c, data_array)
  k = kernel_vector(warp_pt, data_array);
  pt = A.' * k + B * warp_pt + c;
end

% kernel vector for 3-dimensional case
function k = kernel_vector(warp_pt, data_array)
  [d, n] = size(data_array);
  k = zeros(n, 1);
  for i = 1:n
    r = norm(warp_pt - data_array(:,i));
    if r == 0
      k(i) = 0;
    else
        if d == 2
            k(i) = (r^2) * log(r);
        elseif d == 3
            k(i) = -r;
        end
    end
  end
end
