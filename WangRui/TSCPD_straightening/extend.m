function exA = extend(a)
exA = [(1 : size(a, 1))', a];
exA(:, 1) = exA(:, 1) * 10;
end