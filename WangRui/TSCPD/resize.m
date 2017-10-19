function new_test_goal_q = resize(test_goal_q, n)
% try two diff methods:
% cubic spline interpolation, or
% linear interpolation ! use linear interpolation to avoid wave shape
MAG = 10; % magnitude of magnification in x coord
new_test_goal_q = test_goal_q; % pre-allocate space
[~, t2] = sort(test_goal_q(1, :));
test_goal_q = test_goal_q(:, t2); % sort original matrix according to x coord
l = 1; % left bound
m = size(test_goal_q, 1);
test_goal_q(m + 1, :) = [0, 0]; % prevent index-out-of-range in for-loop
for i = 1 : n
     while (l <= m) && (test_goal_q(l, 1) <= i*MAG)
         l = l + 1;
     end
     if l > 1 && l ~= m + 1
         lbar = i*MAG - test_goal_q(l - 1, 1);
         lval = test_goal_q(l - 1, 2);
         rbar = test_goal_q(l, 1) - i*MAG;
         rval = test_goal_q(l, 2);
         new_test_goal_q(i, 1) = i*MAG;
         new_test_goal_q(i, 2) =  (lval * lbar + rval * rbar) / (lbar + rbar); % weighted mean
     elseif l == 1
         new_test_goal_q(i, 1) = i*MAG;
         new_test_goal_q(i, 2) = test_goal_q(1, 2);
     else % l == m + 1
         new_test_goal_q(i, 1) = i*MAG;
         new_test_goal_q(i, 2) = test_goal_q(m, 2);
     end
end
end