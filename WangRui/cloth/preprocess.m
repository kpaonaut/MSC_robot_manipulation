function upperEdge = preprocess(x, y)
%% Get profile
N = size(x, 1);
q = zeros(N, N);
deltaQ = zeros(N, 1);
isEdge = zeros(N, 1);
for i = 1:N
    for j = 1:N
        q(i, j) = atan2(x(j)-x(i), y(j)-y(i)) / pi * 180;
    end
%     if i == 1, qSorted = sort(q(1, 2:N));
%     elseif i == N, qSorted = sort(q(N, 1:N-1));
%     else qSorted = sort([q(i, 1:i-1), q(i, i+1:N)]);
%     end
    qSorted = sort([q(i, 1:i-1), q(i, i+1:N)]);
    for j = 1:N-2
        deltaQ(j) = qSorted(j+1) - qSorted(j);
        if deltaQ(j) > 150
            isEdge(i) = 1;
            break
        end
    end
    deltaQ(N-1) = qSorted(1)+360-qSorted(N-1);
    if deltaQ(N-1) > 140
        isEdge(i) = 1;
    end
end
edgeI = find(isEdge == 1)';
% scatter(x(edgeI, 1), y(edgeI, 1), 'o');
% hold on
%% Make profile clean(points too close will be eliminated)
M = size(edgeI, 2);
deleted = zeros(M, 1);
for i = 1:M
    for j = i+1:M
        diff = sqrt((x(edgeI(i)) - x(edgeI(j)))^2 + (y(edgeI(i)) - y(edgeI(j)))^2);
        if diff < 0.005
            deleted(j) = 1;
        end
    end
end
edge = [];
for i = 1:M
    if deleted(i) == 0
        edge = [edge, edgeI(i)];
    end
end
scatter(x(edge, 1), y(edge, 1), 'x');
hold on
%% Find the edge we are interested in.(the upper-most edge in kinect frame)
% ul = upper-left   ur = upper-right  ll = lower-left  lr = lower-right,
% all in kinect frame
K = size(edge, 2);
dist_ul = [x(edge, 1) + 0.5, y(edge, 1) - 0.5];
[~, i_ul] = min(dist_ul(:, 1).^2 + dist_ul(:, 2).^2);
i_ul = edge(i_ul); % point No. i_ul is upper-left

dist_ur = [x(edge, 1) - 0.5, y(edge, 1) - 0.5];
[~, i_ur] = min(dist_ur(:, 1).^2 + dist_ur(:, 2).^2);
i_ur = edge(i_ur);

idx = i_ul;
upperEdge = [];
while idx ~= i_ur
    upperEdge = [upperEdge; idx]; % column
    xyDiff = [x(edge) - x(idx)].^2 + [y(edge) - y(idx)].^2;
    [~, I] = sort(xyDiff);
    for i = 1:10
        if x(edge(I(i))) > x(idx) && abs(x(edge(I(i))) - x(idx)) > abs(y(edge(I(i))) - y(idx))
            idx = edge(I(i));
            break;
        end
    end
end
upperEdge = [upperEdge; i_ur];
plot(x(upperEdge), y(upperEdge)); % plot the upperEdge to check!