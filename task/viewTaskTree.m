% Wenjie Chen, FANUC Corporation, 2016/05/19
% Plot the task tree and return the tree object and figure handle

function [taskTree, tree_fh] = viewTaskTree(graphData, Type_set)

taskTree = view(biograph(graphData, Type_set));
child_handles = allchild(0);
handle_names = get(child_handles, 'Name');
tree_fh = child_handles(strncmpi('Biograph Viewer', handle_names, 15));

end