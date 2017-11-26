function recoverPlot(stPt, q, LENGTH, opt)
coord = zeros(size(q, 1), 2);
for i = 1:size(q, 1) % q: column vector
    gp = integrate(stPt, q, i, LENGTH, opt); % calculate where the gripping point (x, y) on rope should be
    coord(i, 1) = gp(1);
    coord(i, 2) = gp(2);
end
plot(coord(:, 1), coord(:, 2));
xlim([-0.5, 0.5])
end