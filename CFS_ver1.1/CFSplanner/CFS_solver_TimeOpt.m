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
function [zref, data] = CFS_solver_TimeOpt(robot, status, xref)

time = [];
options = optimoptions(@quadprog,'Display','off'); %final

nlink = robot.nlink;
if strcmp(status.dualmotion, 'yes')
    Nlink = nlink*2;
else
    Nlink = nlink;
end
n1_old = 0;
horizon = size(xref,1)/Nlink;

tref = ones(horizon,1)*robot.delta_t;
uref = ones(horizon*Nlink,1);
zref = [tref; uref];

dt_bound = 0.5;
ub = [robot.delta_t*dt_bound*ones(horizon,1); 0.25*ones(Nlink*horizon,1)];
lb = [0.008*ones(horizon,1); -0.25*ones(Nlink*horizon,1)];

Ht = eye(horizon);
Hu = eye(horizon*Nlink);
ft = zeros(horizon,1);
fu = zeros(horizon*Nlink,1);

He = blkdiag(1*Ht, 1/Nlink*Hu);
fe = [ft; fu];

for k=1:10
    disp(['Iteration',num2str(k)]);
    tic
    Lstack=[]; Sstack=[];

    for i=3:horizon %-2        
        if  i > 2;
            xk2 = xref((i-1)*Nlink+1:(i-0)*Nlink); % x_{k+1}
            xk1 = xref((i-2)*Nlink+1:(i-1)*Nlink); % x_k
            xk0 = xref((i-3)*Nlink+1:(i-2)*Nlink); % x_{k-1}
            tk1 = zref(i);            % t_k
            tk0 = zref(i-1);          % t_{k-1}
            uk1 = zref(length(tref)+(i-2)*Nlink+1:length(tref)+(i-1)*Nlink); % u_k
            
            lx0 = tk1*eye(Nlink);
            lx1 = -(tk1+tk0)*eye(Nlink);
            lx2 = tk0*eye(Nlink);
            lt0 = xk2 - xk1 - tk1^2*uk1;
            lt1 = -xk1 + xk0 - 2*tk1*tk0*uk1;
            lu1 = -tk1^2*tk0*eye(Nlink);
            
            s0 = tk0.*(xk2-xk1) - tk1.*(xk1-xk0) - (tk1^2*tk0).*uk1;
            s = s0 - [lx0 lx1 lx2 lt0 lt1 lu1]*[xk0; xk1; xk2; tk0; tk1; uk1];
            l = [zeros(Nlink,i-2),-lt0, -lt1, zeros(Nlink,horizon-i),...
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
            l = [zeros(Nlink,i-2),-lt0, -lt1, zeros(Nlink,horizon-i),...
                 zeros(Nlink,(i-1)*Nlink), -lu1, zeros(Nlink,(horizon-i)*Nlink)];
            Sstack=[Sstack;s];
            Lstack=[Lstack;l];
        end        
    end

    time(k,1)=toc;
    tic;
    znew = quadprog(He, fe, Lstack, Sstack, [], [], lb, ub, zref, options);

    while isempty(znew)
        dt_bound = dt_bound+0.05;
        ub = [robot.delta_t*dt_bound*ones(horizon,1); 1*ones(Nlink*horizon,1)];
        znew = quadprog(He, fe, Lstack, Sstack, [], [], lb, ub, zref, options);
    end
    
%%%%%%%%%%%%%%%%
    tref = znew(1:horizon);
    
    vref = zeros(length(xref),1);
    aref = zeros(length(xref),1);
    for i = 1:Nlink 
        vref(i:Nlink:end) = diff2(xref(i:Nlink:end))./tref;
        aref(i:Nlink:end) = diff2(vref(i:Nlink:end))./tref;
    end
    zref = [tref; aref];
    
    Lstack=[]; Sstack=[];

    for i=3:horizon %-2        
        if  i > 2;
            xk2 = xref((i-1)*Nlink+1:(i-0)*Nlink); % x_{k+1}
            xk1 = xref((i-2)*Nlink+1:(i-1)*Nlink); % x_k
            xk0 = xref((i-3)*Nlink+1:(i-2)*Nlink); % x_{k-1}
            tk1 = zref(i);            % t_k
            tk0 = zref(i-1);          % t_{k-1}
            uk1 = zref(length(tref)+(i-2)*Nlink+1:length(tref)+(i-1)*Nlink); % u_k
            
            lx0 = tk1*eye(Nlink);
            lx1 = -(tk1+tk0)*eye(Nlink);
            lx2 = tk0*eye(Nlink);
            lt0 = xk2 - xk1 - tk1^2*uk1;
            lt1 = -xk1 + xk0 - 2*tk1*tk0*uk1;
            lu1 = -tk1^2*tk0*eye(Nlink);
            
            s0 = tk0.*(xk2-xk1) - tk1.*(xk1-xk0) - (tk1^2*tk0).*uk1;
            s = s0 - [lx0 lx1 lx2 lt0 lt1 lu1]*[xk0; xk1; xk2; tk0; tk1; uk1];
            l = [zeros(Nlink,i-2),-lt0, -lt1, zeros(Nlink,horizon-i),...
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
            l = [zeros(Nlink,i-2),-lt0, -lt1, zeros(Nlink,horizon-i),...
                 zeros(Nlink,(i-1)*Nlink), -lu1, zeros(Nlink,(horizon-i)*Nlink)];
            Sstack=[Sstack;s];
            Lstack=[Lstack;l];
        end        
    end
    znew = quadprog(He, fe, Lstack, Sstack, [], [], lb, ub, zref, options);

    %%%%%%%%%%%%%%%%%%    
    
    time(k,2)=toc;

    n1 = norm(zref-znew);
    if norm(zref-znew)<0.01*horizon || norm(n1 - n1_old) <0.001
        disp(strcat('Converged at step',num2str(k)));
        disp('Time profile:');
        disp(time);
        disp('Total time:');
        disp(sum(sum(time)));
        
        zref = znew; 
        data.time = time;
        data.dt = znew(1:horizon);
        data.totaltime = sum(sum(time));
%         dt_bound
        return;
    end

    zref = znew;
    data.dt = znew(1:horizon);
    n1_old = n1;
    
end
disp('Maximum Iter Reached without Convergence');
disp('Total time:');
disp(sum(sum(time)));

data.time = time;
data.totaltime = sum(sum(time));


end