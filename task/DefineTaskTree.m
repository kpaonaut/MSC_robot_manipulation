function graphData = DefineTaskTree
% malab is not good for implementing a graph
% you may want to use python or Java when the task becomes more complicated


% 1: double folds, 2: fold, 3: line, 4: tie
E = [ 2, 3;     % unfold 
      2, 4;     % tie 
      4, 2;     % un-tie
      3, 2;     % fold 
      3, 1;     % double-fold
      1, 3;     % un-double-fold
      4, 3;     % un-tie
      1, 2;     % un-double-fold
      2, 1;     % double-fold
      ];    
     
nNode = max(max(E));
E = [E; nNode, nNode]; % this is to make the sparse matrix a square matrix
W = [ones(1,size(E,1)-1),0]; % weight

graphData = sparse(E(:,1),E(:,2),W);

end
