    %% visualize the warping of the original training rope and the test rope
    %disp('Please double check which robot''s motion needs to be warped!');
    fig2_handle = figure(2);
    set(fig2_handle, 'position', [0.962 0.562 0.958 0.434]);
    orig_fig = subplot(1,2,1);
    scatter(points_Test_W(:, 1), points_Test_W(:, 2), 'ro');
    hold on
    scatter(points_W{step}(:, 1), points_W{step}(:, 2), 'kx');
    scatter(points_W{2}(:, 1), points_W{2}(:, 2), 'b','filled');
    hold off
    title('Before Warping'); % plot the original rope 2-D shape
    
    warp_fig = subplot(1,2,2);
    scatter(A(:, 1), A(:, 2), 'ro');
    hold on
    scatter(points_W{step}(:, 1), points_W{step}(:, 2), 'kx');
    scatter(B(:, 1), B(:, 2), 'b','filled');
    title('After Warping'); % plot the test rope
    legend(['Training Rope'], ['Testing Rope'], ['Goal'])
    legend('location', 'southwest')   
    draw_grid([0.1 0.5], [1.1 -0.7], warp, 20, orig_fig, warp_fig)
% 
%     legend(['Training Rope'], ['Testing Rope'], ['Goal'])
%     legend('location', 'southwest')    