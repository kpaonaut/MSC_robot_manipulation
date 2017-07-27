% Wenjie Chen, FANUC Corporation, 2016/03/23
% Get current cable data and its classified state

function cableData = getCurrentState(k2, imSize, data_mean, net, Type_set)

PrintMsg('peek state');

% Get cable data and run classification
cableType = -1;  % initialize to unknown state
while cableType < 0
    cableData = cam2pcl(k2);
    cableData.cnnData = getData4Classfier(cableData.ptCld, imSize, data_mean);
    cableType = runCNNClassifer(net, cableData.cnnData, Type_set);
end
cableData.type = cableType;

closefigs;

end