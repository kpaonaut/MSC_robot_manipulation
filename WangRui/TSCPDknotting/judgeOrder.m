% This function judges which end of the rope is closer to [-500, -500], and
% assigns that end as the beginning index of the rope
% Namely the upper-left corner is beginning
function tf = judgeOrder(pts)
d1 = norm(pts(1, 1:2) - [-500, -500]);
d2 = norm(pts(end, 1:2) - [-500, -500]);
if d1 > d2 % needs reverse
    tf = 1;
else
    tf = 0;
end
end