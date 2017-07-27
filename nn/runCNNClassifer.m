% Wenjie Chen, FANUC Corporation, 2016/03/23
% Run classification to get cable type

function cableType = runCNNClassifer(net, data, Type_set)

res = vl_simplenn(net, data, [], [], 'mode', 'test');
scores = squeeze(gather(res(end).x));
[bestScore, best] = max(scores);

if bestScore <= 0.75
    PrintMsg('state unclear');
    cableType = -1;     % Not successful
else
    cableType = best;
    PrintMsg('current state', Type_set{best});
end

end