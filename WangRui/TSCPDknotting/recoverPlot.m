function recoverPlot(stPt, q, LENGTH, opt)
coord = zeros(size(q, 1), 2);
for i = 1:size(q, 1) % q: column vector
    gp = integrate(stPt, q, i, LENGTH, opt); % calculate where the gripping point (x, y) on rope should be
    coord(i, 1) = gp(1);
    coord(i, 2) = gp(2);
end
scatter(coord(:, 1), coord(:, 2), '*', 'r'); % for test
% scatter(coord(:, 1), coord(:, 2);
xlim([-0.1, 0.5])
ylim([-0.1, 0.35])
end