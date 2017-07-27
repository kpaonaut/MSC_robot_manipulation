function warped = warp_pts(pts, warp)
  %WARP_PTS Warps a matrix of points
  %
  % warp_pts(pts, warp)
  %
  % Given a matrix of points pts, where each column represents one point, as
  % well as a warping function warp, returns a matrix of warped points.
  % Corresponding columns represent corresponding points. That is:
  %
  %     warped(:,i) == warp(pts(:,i))

  warped = zeros(size(pts));
  if ~isnumeric(warp(pts(:,1)))
    warped = sym(warped);
  end
  for i = 1:size(pts, 2)
    warped(:,i) = warp(pts(:,i));
  end
end
