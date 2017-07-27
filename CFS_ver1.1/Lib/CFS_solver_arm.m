function [xref,dstack] = CFS_solver_arm(robot,obs,x0)
xref = x0;
time = [];
options = optimoptions(@quadprog,'Display','off');

[nobs,horizon] = size(obs);
nlink = robot.nlink;
%% Check if the solution is good enough
n1_old = 0;
for k=1:10
    disp(['Iteration',num2str(k)]);
    tic
    Lstack=[];Sstack=[]; dstack = [];
    for i=1:horizon
        for j=1:nobs
        Diff=zeros(size(xref));
        
        for s=1:nlink
            fun = @(x) dist_arm_3D([xref((i-1)*nlink+1:(i-1)*nlink+s-1);x;xref((i-1)*nlink+s+1:i*nlink)],robot.DH(1:nlink,:),robot.base,obs{j,i},robot.cap)-robot.margin;
%             fun = @(x) dist_arm_3D_Dual([xref((i-1)*nlink+1:(i-1)*nlink+s-1);x;xref((i-1)*nlink+s+1:i*nlink)],obs{j,i}, robot)-robot.margin;
            [Diff((i-1)*nlink+s),~]=derivest(fun,xref((i-1)*nlink+s),'Vectorized','no');
        end

%         Hess=hessian(@(x) dist_arm_3D(x,robot.DH(1:nlink,:),robot.base,obs{j,i},robot.cap), xref((i-1)*nlink+1:i*nlink)  );
        dd = dist_arm_3D(xref((i-1)*nlink+1:i*nlink),robot.DH(1:nlink,:),robot.base,obs{j,i},robot.cap);
%         dd = dist_arm_3D_Dual(xref((i-1)*nlink+1:i*nlink),obs{j,i}, robot);
        s= dd -robot.margin-Diff'*xref;
        l=-Diff';
        
%         [E,lambda]=eig(Hess);
%         for m=1:size(lambda,1)
%             if lambda(m,m)<0
%                 s = s+lambda(m,m)/size(lambda,1)/10;
%                 flag = 'yes';
%             end
%         end
        
        Sstack=[Sstack;s];
        Lstack=[Lstack;l];
        dstack = [dstack; dd];
        end
    end
    Sstack = [Sstack;robot.opt.b];
    Lstack = [Lstack;robot.opt.A];
    
    time(k,1)=toc;
    tic;
    xnew = quadprog(robot.opt.H,robot.opt.f,Lstack,Sstack,robot.opt.Aeq,robot.opt.beq,robot.opt.lb,robot.opt.ub,x0,options);
    time(k,2)=toc;
    n1 = norm(xref-xnew)
    if norm(xref-xnew)<0.01*horizon || norm(n1 - n1_old) <0.001
        disp(strcat('Converged at step',num2str(k)));
%         disp('Solution:');
%         disp(xnew);
        disp('Time profile:');
        disp(time);
        disp('Total time:');
        disp(sum(sum(time)));
        xref=xnew;
        return;
    end
    xref=xnew;
    n1_old = n1;
end
disp('Maximum Iter Reached without Convergence');
disp('Total time:');
disp(sum(sum(time)));
end