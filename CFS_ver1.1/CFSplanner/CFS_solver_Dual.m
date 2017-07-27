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
function [xref, data] = CFS_solver_Dual(robot,robot2, status, obs, x0, varargin)

xref = x0;
time = [];
options = optimoptions(@quadprog,'Display','off');

nlink = robot.nlink;
if strcmp(status.dualmotion,'yes')
    Nlink = nlink*2;
else
    Nlink = nlink;
end
n1_old = 0;

horizon = size(xref,1)/Nlink;

flag_move = false;

if ~isempty(varargin);    ObsMotion = varargin{1};  end

for i = length(obs): -1 : 1
    if strcmp(obs{i}.ismove, 'yes')
        flag_move = true;
        continue
    end
end
%% Check if the solution is good enough

for k=1:10
    disp(['Iteration',num2str(k)]);
    tic
    Lstack=[];Sstack=[]; dstack = [];

    for i=3:horizon-2
        Diff=zeros(size(xref));
        for s=1:Nlink
            if flag_move
%                for j = length(obs): -1 : 1
%                     if strcmp(obs{j}.ismove, 'yes')
%                         obs{j} = updateMovingObs(obs{j});
%                     end
%                end
                obs{end-7} = updateMovingObs(obs{end-7}, [ObsMotion(1:3,i), ObsMotion(4:6,i)]);
                obs{end-6} = updateMovingObs(obs{end-6}, [ObsMotion(4:6,i), ObsMotion(7:9,i)]);
%                 obs{end-9:end} = UpdateHuCap(obs{end-9:end}, ObsMotion(:,i)'); % This only for one human (Update 10 links one time)
            end
            if strcmp(status.dualmotion,'yes')
                fun = @(x) CFSdistfun([xref((i-1)*Nlink+1:(i-1)*Nlink+s-1);x;xref((i-1)*Nlink+s+1:i*Nlink)],obs, robot, robot2);%-robot.margin; % for static obs only!
            else
                fun = @(x) CFSdistfun([xref((i-1)*Nlink+1:(i-1)*Nlink+s-1);x;xref((i-1)*Nlink+s+1:i*Nlink)],obs, robot);%-robot.margin;
            end
            Diff((i-1)*Nlink+s) = derivest(fun, xref((i-1)*Nlink+s), 'Vectorized', 'no', 'FixedStep', 1e-3);

        end

%%%%      Calculate Hessian will be more acurate, but much slower
%         Hess=hessian(@(x) dist_arm_3D(x,robot.DH(1:Nlink,:),robot.base,obs{j,i},robot.cap), xref((i-1)*Nlink+1:i*Nlink)  );

        if strcmp(status.dualmotion,'yes')
            dist = CFSdistfun(xref((i-1)*Nlink+1:i*Nlink),obs, robot, robot2); % for static obs only!
        else
            dist = CFSdistfun(xref((i-1)*Nlink+1:i*Nlink),obs, robot);
        end
        
        s= dist -Diff'*xref - robot.margin;
        l=-Diff';
        
%%%%      Calculate Hessian will be more acurate, but much slower        
%         [E,lambda]=eig(Hess);
%         for m=1:size(lambda,1)
%             if lambda(m,m)<0
%                 s = s+lambda(m,m)/size(lambda,1)/10;
%                 flag = 'yes';
%             end
%         end
        
        Sstack=[Sstack;s];
        Lstack=[Lstack;l];
        dstack = [dstack; dist];

        
    end
    
    Sstack = [Sstack;robot.opt.b];
    Lstack = [Lstack;robot.opt.A];
    
    time(k,1)=toc;
    tic;
    xnew = quadprog(robot.opt.H,robot.opt.f,Lstack,Sstack,robot.opt.Aeq,robot.opt.beq,robot.opt.lb,robot.opt.ub,x0,options);
    time(k,2)=toc;
    if isempty(xnew)
        xnew;
    end
    n1 = norm(xref-xnew);
    if norm(xref-xnew)<0.01*horizon || norm(n1 - n1_old) <0.001
        disp(strcat('Converged at step',num2str(k)));
%         disp('Solution:');
%         disp(xnew);
        disp('Time profile:');
        disp(time);
        disp('Total time:');
        disp(sum(sum(time)));
        xref=xnew;
        
        data.time = time;
        data.totaltime = sum(sum(time));
        data.dist = dstack;

        return;
    end
    xref=xnew;
    n1_old = n1;
end
disp('Maximum Iter Reached without Convergence');
disp('Total time:');
disp(sum(sum(time)));

data.time = time;
data.totaltime = sum(sum(time));
data.dist = dstack;

end