%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%        Convex Feasible Set Algorithm Solver
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%  Based on Changliu's CFS algorithm 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Input:
%  robot   : [1x1 cell], robot No.1 parameters, if it is dual motion, robot{1}
%             contains the optimization parameters, otherwise, the moving 
%             robot contains them.
%  robot2  : [1x1 cell], robot No.2 parameters
%  status  : [1x1 struct], indicates the robots motion
%  obs     : [1xN cell], blocks or tools in the workspace
%  x0      : [Nstate*horizon x 1], vectorized path reference. "horizon" is 
%            the downsampled timestep, and "Nstate" is 6 or 12, which depends  
%            on dual motion or not.
% 
%  Output:
%  xref: [Nstate*horizon x 1], the optimized path. 
%  data: [1x1 struct], store the CFS computation time and distance over the
%        horizon.
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [xref, data] = CFS_solver_Dual(robot,robot2, status, obs, x0)
extend = false; %true or false;
xref = x0;

time = [];
options = optimoptions(@quadprog,'Display','final');

nlink = robot.nlink;
if strcmp(status.dualmotion,'yes')
    Nlink = nlink*2;
else
    Nlink = nlink;
end
n1_old = 0;
horizon = size(xref,1)/Nlink;

if extend == true
    tref = ones(horizon,1)*robot.delta_t;
    uref = ones(horizon*Nlink,1);
    zref = [xref; tref; uref];
    robot.opt.Aeq = [robot.opt.Aeq, zeros(Nlink*4, horizon*(Nlink+1))];
    robot.opt.ub = [robot.opt.ub; robot.delta_t*ones(horizon,1); 2*ones(Nlink*horizon,1)];
    robot.opt.lb = [robot.opt.lb; 0.008*ones(horizon,1); -2*ones(Nlink*horizon,1)];
end


%% Check if the solution is good enough

for k=1:10
    disp(['Iteration',num2str(k)]);
    tic
    Lstack=[];Sstack=[]; dstack = [];

    for i=3:horizon %-2
        Diff=zeros(size(xref));
        for s=1:Nlink
            if strcmp(status.dualmotion,'yes')
                fun = @(x) CFSdistfun([xref((i-1)*Nlink+1:(i-1)*Nlink+s-1);x;xref((i-1)*Nlink+s+1:i*Nlink)],obs, robot, robot2)-robot.margin; % for static obs only!
            else
                fun = @(x) CFSdistfun([xref((i-1)*Nlink+1:(i-1)*Nlink+s-1);x;xref((i-1)*Nlink+s+1:i*Nlink)],obs, robot)-robot.margin;
            end
            Diff((i-1)*Nlink+s) = derivest(fun, xref((i-1)*Nlink+s), 'Vectorized', 'no', 'FixedStep', 1e-3);
        end
        
        if strcmp(status.dualmotion,'yes')
            dist = CFSdistfun(xref((i-1)*Nlink+1:i*Nlink),obs, robot, robot2); % for static obs only!
        else
            dist = CFSdistfun(xref((i-1)*Nlink+1:i*Nlink),obs, robot);
        end
        
        s= dist -Diff'*xref - robot.margin;
        l=-Diff';
        
%%%%      Calculate Hessian will be more acurate, but much slower
%         Hess=hessian(@(x) dist_arm_3D(x,robot.DH(1:Nlink,:),robot.base,obs{j,i},robot.cap), xref((i-1)*Nlink+1:i*Nlink)  );

%%%%      Calculate Hessian will be more acurate, but much slower        
%         [E,lambda]=eig(Hess);
%         for m=1:size(lambda,1)
%             if lambda(m,m)<0
%                 s = s+lambda(m,m)/size(lambda,1)/10;
%                 flag = 'yes';
%             end
%         end
        if extend == true
            l = [l, zeros(1,horizon*(1+Nlink))];
        end
        
        Sstack=[Sstack;s];
        Lstack=[Lstack;l];
        dstack = [dstack; dist];
        
        if extend == true && i > 2;
            xk2 = xref((i-1)*Nlink+1:(i-0)*Nlink); % x_{k+1}
            xk1 = xref((i-2)*Nlink+1:(i-1)*Nlink); % x_k
            xk0 = xref((i-3)*Nlink+1:(i-2)*Nlink); % x_{k-1}
            tk1 = zref(length(xref)+i);            % t_k
            tk0 = zref(length(xref)+i-1);          % t_{k-1}
            uk1 = zref(length(xref)+length(tref)+(i-2)*Nlink+1:length(xref)+length(tref)+(i-1)*Nlink); % u_k
            
            lx0 = tk1*eye(Nlink);
            lx1 = -(tk1+tk0)*eye(Nlink);
            lx2 = tk0*eye(Nlink);
            lt0 = xk2 - xk1 - tk1^2*uk1;
            lt1 = -xk1 + xk0 - 2*tk1*tk0*uk1;
            lu1 = -tk1^2*tk0*eye(Nlink);
            
            s0 = tk0.*(xk2-xk1) - tk1.*(xk1-xk0) - (tk1^2*tk0).*uk1;
            s = s0 - [lx0 lx1 lx2 lt0 lt1 lu1]*[xk0; xk1; xk2; tk0; tk1; uk1];
            l = [zeros(Nlink,(i-3)*Nlink), -lx0, -lx1, -lx2, zeros(Nlink,(horizon-i)*Nlink),...
                 zeros(Nlink,i-2),-lt0, -lt1, zeros(Nlink,horizon-i),...
                 zeros(Nlink,(i-1)*Nlink), -lu1, zeros(Nlink,(horizon-i)*Nlink)];
            Sstack=[Sstack;s];
            Lstack=[Lstack;l];
            
            lx0 = tk1*eye(Nlink);
            lx1 = -(tk1+tk0)*eye(Nlink);
            lx2 = tk0*eye(Nlink);
            lt0 = xk2 - xk1 + tk1^2*uk1;
            lt1 = -xk1 + xk0 + 2*tk1*tk0*uk1;
            lu1 = tk1^2*tk0*eye(Nlink);
            
            s0 = tk0.*(xk2-xk1) - tk1.*(xk1-xk0) + (tk1^2*tk0).*uk1;
            s = s0 - [lx0 lx1 lx2 lt0 lt1 lu1]*[xk0; xk1; xk2; tk0; tk1; uk1];
            l = [zeros(Nlink,(i-3)*Nlink), -lx0, -lx1, -lx2, zeros(Nlink,(horizon-i)*Nlink),...
                 zeros(Nlink,i-2),-lt0, -lt1, zeros(Nlink,horizon-i),...
                 zeros(Nlink,(i-1)*Nlink), -lu1, zeros(Nlink,(horizon-i)*Nlink)];
            Sstack=[Sstack;s];
            Lstack=[Lstack;l];
        end
        
    end
    
    Sstack = [Sstack;robot.opt.b];
    Lstack = [Lstack;robot.opt.A];

    time(k,1)=toc;
    tic;
    if extend == true
        znew = quadprog(robot.opt.He,robot.opt.fe,Lstack,Sstack,robot.opt.Aeq,robot.opt.beq,robot.opt.lb,robot.opt.ub,zref,options);
        xnew = znew(1:Nlink*horizon);
%         tnew = znew( Nlink*horizon+1 : Nlink*horizon+horizon);
%         unew = znew((Nlink+1)*horizon+1 : end);
    else
        xnew = quadprog(robot.opt.H,robot.opt.f,Lstack,Sstack,robot.opt.Aeq,robot.opt.beq,robot.opt.lb,robot.opt.ub,xref,options);
    end
    time(k,2)=toc;

    n1 = norm(xref-xnew);
    if norm(xref-xnew)<0.01*horizon || norm(n1 - n1_old) <0.001
        disp(strcat('Converged at step',num2str(k)));
%         disp('Solution:');
%         disp(xnew);
        disp('Time profile:');
        disp(time);
        disp('Total time:');
        disp(sum(sum(time)));
        
        xref = xnew;
        if extend == true
            zref = znew;
            data.dt = znew(length(xref)+1:length(xref)+horizon);
        end
        data.time = time;
        data.totaltime = sum(sum(time));
        data.dist = dstack;
        
        
        return;
    end
    xref=xnew;
    if extend == true
        zref = znew;
        data.dt = znew(length(xref)+1:length(xref)+horizon);
    end
    n1_old = n1;
end
disp('Maximum Iter Reached without Convergence');
disp('Total time:');
disp(sum(sum(time)));

data.time = time;
data.totaltime = sum(sum(time));
data.dist = dstack;

end