tmpx = [points_W{2}];
fig2_handle = figure(2);
orig_fig = subplot(2,2,1);
scatter(tmpx(:, 1), tmpx(:, 2), 'filled');
axis([0.5 1 -0.6 0.4]);
title('Training Rope');

warp_fig = subplot(2,2,2);
points_train_q = getQ(points_W{2}(:, 1 : 2));
plot(points_train_q);
xlabel('Node No.');ylabel('Tangent Line Angle')
title('Tangent Information of Training Rope');
axis([0 50 0 100])

points_test_q = PtCld_Test_ts;

subplot(2,2,4);
plot( points_test_q );
title('Tangent Information of Testing Goal');
xlabel('Node No.');ylabel('Tangent Line Angle')
axis([0 50 0 100])

subplot(2,2,3);
for i=1:size(points_test_q, 1)
reconst(i,:) = integrate(tmpx(1, 1:2), points_test_q, i, 0.02, 1);
end
scatter(reconst(:, 1), reconst(:, 2), 'filled');
axis([0.5 1 -0.6 0.4]);
title('Reconstructed Goal');
