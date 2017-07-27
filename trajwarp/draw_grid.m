function draw_grid(upper_left, bottom_right, warp, num_segments, orig_fig, ...
    warp_fig)
%DRAW_GRID Visualize a regular grid before and after warping
%
% draw_grid(upper_left, bottom_right, warp, num_segments, orig_fig, warp_fig)
%
% Given a grid defined by upper_left, bottom_right, and num_segments, this
% function draws two plots: one into orig_fig representing the untransformed
% grid, and one into warp_fig representing the grid after the warp function
% has been applied to it.
%
% The grid's boundaries are defined by upper_left and bottom_right, which mark
% the upper-left and bottom-right corners respectively. The intermediate
% gridlines are regularly spaced, and their number is determined by
% num_segments. For example, a num_segments value of 2 will produce a grid
% that has 4 squares.
%

%% TODO: Ability to omit one of the figures

orig_lines = {};
warp_lines = {};
n = 1;
for x = linspace(upper_left(1), bottom_right(1), num_segments+1)
    orig_lines{n} = [x x; upper_left(2) bottom_right(2)];
    warp_lines{n} = warp_line([x upper_left(2)], [x bottom_right(2)], warp, ...
        100);
    n = n + 1;
end
for y = linspace(upper_left(2), bottom_right(2), num_segments+1)
    orig_lines{n} = [upper_left(1) bottom_right(1); y y];
    warp_lines{n} = warp_line([upper_left(1) y], [bottom_right(1) y], warp, ...
        100);
    n = n + 1;
end
%   figure(orig_fig);
subplot(orig_fig);
hold on;
for i = 1:n - 1
    pts = orig_lines{i};
    plot(pts(1,:), pts(2,:));
end
hold off;
%figure(warp_fig);
subplot(warp_fig);
hold on;
for i = 1:n - 1
    pts = warp_lines{i};
    plot(pts(1,:), pts(2,:));
end
hold off;
end
