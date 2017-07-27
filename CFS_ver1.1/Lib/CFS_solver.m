function xref = CFS_solver(H,f,phi,Cor,A,b,Aeq,beq,lb,ub,x0)
xref = x0;
time = [];
options = optimoptions(@quadprog,'Display','off');
VSelect = @(V,IDX) V(IDX);
%% Check if the solution is good enough

for k=1:10
    disp(['Iteration',num2str(k)]);
    tic
    Lstack=[];Sstack=[];
    for i=1:size(phi(xref),1)
        Diff=zeros(size(xref));
        for s=Cor{i}
            [Diff(s),~]=derivest(@(x) VSelect(phi([xref(1:s-1);x;xref(s+1:end)]),i),xref(s),'Vectorized','no');
        end
        
        % Hess=hessian(@(x) phi([xref(1:min(Cor{i})-1);x;xref(max(Cor{i})+1:end),xref);
        
        s=VSelect(phi(xref),i)-Diff'*xref;
        l=-Diff';
        
%         [E,lambda]=eig(Hess);
%         for m=1:size(lambda,1)
%             if lambda(m,m)<0
%                 s = s+lambda(m,m)/size(lambda,1);
%                 flag = 'yes';
%             end
%         end
        
        Sstack=[Sstack;s];
        Lstack=[Lstack;l];
        
    end
    Sstack = [Sstack;b];
    Lstack = [Lstack;A];
    
    time(k,1)=toc;
    tic;
    xnew = quadprog(H,f,Lstack,Sstack,Aeq,beq,lb,ub,x0,options);
    time(k,2)=toc;
    
    if norm(xref-xnew)<0.01
        disp(strcat('Converged at step',num2str(k)));
        disp('Solution:');
        disp(xnew);
        disp('Time profile:');
        disp(time);
        disp('Total time:');
        disp(sum(sum(time)));
        xref=xnew;
        return;
    end
    xref=xnew;
end
disp('Maximum Iter Reached without Convergence');
disp('Total time:');
disp(sum(sum(time)));
end