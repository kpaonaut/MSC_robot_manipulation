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
function [zref, data] = CFS_solver_TimeOpt2(robot, status, xref)

time = [];
options = optimoptions(@quadprog,'Display','final'); %final

nlink = robot.nlink;
if strcmp(status.dualmotion, 'yes')
    Nlink = nlink*2;
else
    Nlink = nlink;
end
n1_old = 0;
horizon = size(xref,1)/Nlink;

tref = ones(horizon,1)*robot.delta_t;
uref = zeros(horizon*Nlink,1);
    for i = 1:horizon
        if i >= 2 && i < horizon
            xk2 = xref((i)*Nlink+1:(i+1)*Nlink); % x_{k+1}
            xk1 = xref((i-1)*Nlink+1:(i)*Nlink); % x_k
            xk0 = xref((i-2)*Nlink+1:(i-1)*Nlink); % x_{k-1}
            v0 = (xk1 - xk0)./tref(i-1);
            v1 = (xk2 - xk1)./tref(i);
            uref((i-1)*Nlink+1:i*Nlink) = (v1-v0)./(tref(i-1)+tref(i))*2;
        elseif i == 1
            xk1 = xref((i-1)*Nlink+1:(i)*Nlink);
            xk2 = xref((i)*Nlink+1:(i+1)*Nlink);
            v1 = (xk2 - xk1)./tref(i);
            v0 = zeros(Nlink,1);
            uref((i-1)*Nlink+1:i*Nlink) = (v1 - v0)./tref(i);
        else % i = horizon
            xk1 = xref((i-1)*Nlink+1:(i)*Nlink);
            xk0 = xref((i-2)*Nlink+1:(i-1)*Nlink);
            v0 = (xk1 - xk0)./tref(i-1);
            v1 = zeros(Nlink,1);
            uref((i-1)*Nlink+1:i*Nlink) = (v1 - v0)./tref(i-1);
        end
    end
% uref = 0*ones(horizon*Nlink,1);
zref = [tref; (uref)]; % abs(uref)

dt_bound = 0.25;
umax = 1;
ub = [robot.delta_t*dt_bound*ones(horizon,1); umax*ones(Nlink*horizon,1)]; 
lb = [0.008*ones(horizon,1); -umax*ones(Nlink*horizon,1)];

Ht = eye(horizon);
Hu = eye(horizon*Nlink);
ft = ones(horizon,1);
fu = zeros(horizon*Nlink,1);

He = blkdiag(1*Ht, 20*Hu);
fe = -1*[ft; fu];

for k=1:10
    disp(['Iteration',num2str(k)]);
    tic
    Lstack=[]; Sstack=[];

    for i=1:horizon %-2        
        if  i >= 2 && i < horizon;
            xk2 = xref((i)*Nlink+1:(i+1)*Nlink); % x_{k+1}
            xk1 = xref((i-1)*Nlink+1:(i)*Nlink); % x_k
            xk0 = xref((i-2)*Nlink+1:(i-1)*Nlink); % x_{k-1}
            tk1 = zref(i);            % t_k
            tk0 = zref(i-1);          % t_{k-1}
            uk1 = zref(length(tref)+(i-1)*Nlink+1:length(tref)+i*Nlink); % u_k
            
            lt0 = -(xk2 - xk1) + (2*tk1*tk0 + tk0^2)/2 * uk1;
            lt1 =  (xk1 - xk0) + (2*tk1*tk0 + tk1^2)/2 * uk1;
            lu1 =  tk1*tk0*(tk1+tk0)/2*eye(Nlink);
%             lt0 = xk2 - xk1 - tk1^2*uk1;
%             lt1 = -xk1 + xk0 - 2*tk1*tk0*uk1;
%             lu1 = -tk1^2*tk0*eye(Nlink);
            
%             s0 = - tk0.*(xk2-xk1) + tk1.*(xk1-xk0) + tk1*tk0*(tk1+tk0)/2.*uk1;
%             s = - s0 - [lt0 lt1 lu1]*[tk0; tk1; uk1];
            s = -1.5*(tk0+tk1)*tk0*tk1*uk1;
            l = [zeros(Nlink,i-2),-lt0, -lt1, zeros(Nlink,horizon-i),...
                 zeros(Nlink,(i-1)*Nlink), -lu1, zeros(Nlink,(horizon-i)*Nlink)];
            Sstack=[Sstack;s];
            Lstack=[Lstack;l];
            
            lt0 =  (xk2 - xk1) + (2*tk1*tk0 + tk0^2)/2 * uk1;
            lt1 = -(xk1 - xk0) + (2*tk1*tk0 + tk1^2)/2 * uk1;
            lu1 =  tk1*tk0*(tk1+tk0)/2*eye(Nlink);            
%             lt0 = xk2 - xk1 + tk1^2*uk1;
%             lt1 = -xk1 + xk0 + 2*tk1*tk0*uk1;
%             lu1 = tk1^2*tk0*eye(Nlink);
            
%             s0 = tk0.*(xk2-xk1) - tk1.*(xk1-xk0) + (tk1^2*tk0).*uk1;
%             s = s0 - [lt0 lt1 lu1]*[ tk0; tk1; uk1];
            s = -1.5*(tk0+tk1)*tk0*tk1*uk1;
            l = [zeros(Nlink,i-2),-lt0, -lt1, zeros(Nlink,horizon-i),...
                 zeros(Nlink,(i-1)*Nlink), -lu1, zeros(Nlink,(horizon-i)*Nlink)];
            Sstack=[Sstack;s];
            Lstack=[Lstack;l];
            
        elseif i == 1           % v0 - v1 + a1 * t1 = 0
            v0 = zeros(Nlink,1); %vinit 
            v1 = xref((i)*Nlink+1:(i+1)*Nlink) - xref((i-1)*Nlink+1:(i-0)*Nlink);            
            tk1 = zref(i);
            uk1 = zref(length(tref)+(i-1)*Nlink+1:length(tref)+i*Nlink);
            lu1 = tk1^2 *eye(Nlink);
            lt1 = v0+2*tk1*uk1;            
            s = -v1+tk1*v0-lt1*tk1;
            l = [zeros(Nlink,i-1),-lt1, zeros(Nlink,horizon-i),...
                 zeros(Nlink,(i-1)*Nlink), -lu1, zeros(Nlink,(horizon-i)*Nlink)];            
            Sstack = [Sstack; s];
            Lstack = [Lstack; l];
            
            lu1 = tk1^2*eye(Nlink);
            lt1 = -v0+2*tk1*uk1;
            s = v1-tk1*v0-lt1*tk1;
            l = [zeros(Nlink,i-1),-lt1, zeros(Nlink,horizon-i),...
                 zeros(Nlink,(i-1)*Nlink), -lu1, zeros(Nlink,(horizon-i)*Nlink)];             
            Sstack = [Sstack; s];
            Lstack = [Lstack; l];
            
        else        % v_{end+1} = v_{end} + a_{end}t_{end} 
            v1 = zeros(Nlink,1); % v_{end+1}
            v0 = xref((i-1)*Nlink+1:(i)*Nlink) - xref((i-2)*Nlink+1:(i-1)*Nlink);            
            tk0 = zref(i);
            uk1 = zref(length(tref)+(i-1)*Nlink+1:length(tref)+i*Nlink);            
            lu1 = tk0^2*eye(Nlink);
            lt0 = -v1+2*tk0*uk1;
            s = v0-tk1*v1-lt0*tk0;
            l = [zeros(Nlink,i-1),-lt0, zeros(Nlink,horizon-i),...
                 zeros(Nlink,(i-1)*Nlink), -lu1, zeros(Nlink,(horizon-i)*Nlink)];            
            Sstack = [Sstack; s];
            Lstack = [Lstack; l];
            
            lu1 = tk0^2*eye(Nlink);
            lt0 = v1+2*tk0*uk1;
            s = -v0+tk0*v1-lt0*tk0;
            l = [zeros(Nlink,i-1),-lt0, zeros(Nlink,horizon-i),...
                 zeros(Nlink,(i-1)*Nlink), -lu1, zeros(Nlink,(horizon-i)*Nlink)];                 
            Sstack = [Sstack; s];            
            Lstack = [Lstack; l];
        end        
    end

    time(k,1)=toc;
    tic;
%     znew = quadprog(He, fe, Lstack, Sstack, [], [], lb, ub, zref, options);
    znew = quadprog(He, fe, Lstack, Sstack, [], [], lb, ub, [], options);
    while isempty(znew)
        dt_bound = dt_bound+0.05;
        ub = [robot.delta_t*dt_bound*ones(horizon,1); 1*ones(Nlink*horizon,1)];
        znew = quadprog(He, fe, Lstack, Sstack, [], [], lb, ub, zref, options);
    end




    %%%%%%%%%%%%%%%%
    tref = znew(1:horizon);
    uref = zeros(horizon*Nlink,1);
    for i = 1:horizon
        if i >= 2 && i < horizon
            xk2 = xref((i)*Nlink+1:(i+1)*Nlink); % x_{k+1}
            xk1 = xref((i-1)*Nlink+1:(i)*Nlink); % x_k
            xk0 = xref((i-2)*Nlink+1:(i-1)*Nlink); % x_{k-1}
            v0 = (xk1 - xk0)./tref(i-1);
            v1 = (xk2 - xk1)./tref(i);
            uref((i-1)*Nlink+1:i*Nlink) = (v1-v0)./(tref(i-1)+tref(i))*2;
        elseif i == 1
            xk1 = xref((i-1)*Nlink+1:(i)*Nlink);
            xk2 = xref((i)*Nlink+1:(i+1)*Nlink);
            v1 = (xk2 - xk1)./tref(i);
            v0 = zeros(Nlink,1);
            uref((i-1)*Nlink+1:i*Nlink) = (v1 - v0)./tref(i);
        else % i = horizon
            xk1 = xref((i-1)*Nlink+1:(i)*Nlink);
            xk0 = xref((i-2)*Nlink+1:(i-1)*Nlink);
            v0 = (xk1 - xk0)./tref(i-1);
            v1 = zeros(Nlink,1);
            uref((i-1)*Nlink+1:i*Nlink) = (v1 - v0)./tref(i-1);
        end
    end
    
%     uref = abs(uref);
    zref = [tref; uref];
       
    %%%%%%%%%%%%%%%%%% 
    
    n1 = norm(zref(1:horizon)-znew(1:horizon));
    if norm(zref(1:horizon)-znew(1:horizon))<0.01*horizon || norm(n1 - n1_old) <0.001
        disp(strcat('Converged at step',num2str(k)));
        disp('Time profile:');
        disp(time);
        disp('Total time:');
        disp(sum(sum(time)));
        
        zref = znew; 
        data.time = time;
        data.dt = znew(1:horizon);
        data.totaltime = sum(sum(time));
        dt_bound
        return;
    end    
%     time(k,2)=toc;
% 
%     n1 = norm(zref-znew);
%     if norm(zref-znew)<0.01*horizon || norm(n1 - n1_old) <0.001
%         disp(strcat('Converged at step',num2str(k)));
%         disp('Time profile:');
%         disp(time);
%         disp('Total time:');
%         disp(sum(sum(time)));
%         
%         zref = znew; 
%         data.time = time;
%         data.dt = znew(1:horizon);
%         data.totaltime = sum(sum(time));
%         dt_bound
%         return;
%     end

%     zref = znew;
%     data.dt = znew(1:horizon);
%     n1_old = n1;
 n1_old = n1;    
end
disp('Maximum Iter Reached without Convergence');
disp('Total time:');
disp(sum(sum(time)));

data.time = time;
data.totaltime = sum(sum(time));
data.dt = znew(1:horizon);

end