load('trainingCloth_1.mat');
scatter(x, y, 'o');

scatter(ts_train(:, 1), ts_train(:, 2), 'o', 'MarkerFaceColor', 'b');
hold on
scatter(ts_test(:, 1), ts_test(:, 2), 'o', 'MarkerFaceColor', 'r');
legend('train','test')
plot(ts_train(:, 1), ts_train(:, 2));
plot(ts_test(:, 1), ts_test(:, 2));