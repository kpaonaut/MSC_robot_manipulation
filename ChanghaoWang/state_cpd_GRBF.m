function [C, W, sigma2, iter, T, L] =state_cpd_GRBF(X, Y, beta, lambda, max_it, tol, viz, outliers, fgt, corresp,sigma2)

[N, D]=size(X); [M, D]=size(Y);

% Initialization
T=Y; iter=0;  ntol=tol+10; W=zeros(M,D);
if ~exist('sigma2','var') || isempty(sigma2) || (sigma2==0), 
    sigma2=(M*trace(X'*X)+N*trace(Y'*Y)-2*sum(X)*sum(Y)')/(M*N*D);
end
sigma2_init=sigma2;


% Construct affinity matrix G
G=cpd_G(Y,Y,beta);


iter=0; ntol=tol+10; L=1;
while (iter<max_it) && (ntol > tol) && (sigma2 > 1e-8) %(sigma2 > 1e-8)
%while (iter<max_it)  && (sigma2 > 1e-8) %(sigma2 > 1e-8)


    L_old=L;
    % Check wheather we want to use the Fast Gauss Transform
    if (fgt==0)  % no FGT
        [P1,Pt1, PX, L]=cpd_P(X,T, sigma2 ,outliers); st='';
    else         % FGT
        [P1, Pt1, PX, L, sigma2, st]=cpd_Pfast(X, T, sigma2, outliers, sigma2_init, fgt);
    end
    
    L=L+lambda/2*trace(W'*G*W);
    ntol=abs((L-L_old)/L);
    disp([' CPD nonrigid ' st ' : dL= ' num2str(ntol) ', iter= ' num2str(iter) ' sigma2= ' num2str(sigma2)]);


    % M-step. Solve linear system for W.

    dP=spdiags(P1,0,M,M); % precompute diag(P)
    W=(dP*G+lambda*sigma2*eye(M))\(PX-dP*Y);
    
    % % same, but solve symmetric system, this can be a bit faster
    % % but can have roundoff errors on idP step. If you want to speed up
    % % use rather a lowrank version: opt.method='nonrigid_lowrank'.
    %
    % idP=spdiags(1./P1,0,M,M); 
    % W=(G+lambda*sigma2*idP)\(idP*PX-Y)

    % update Y postions
    T=Y+G*W;

    Np=sum(P1);sigma2save=sigma2;
    sigma2=abs((sum(sum(X.^2.*repmat(Pt1,1,D)))+sum(sum(T.^2.*repmat(P1,1,D))) -2*trace(PX'*T)) /(Np*D));

    % Plot the result on current iteration
    if viz, cpd_plot_iter(X, T); 
    end;
    iter=iter+1;

end


disp('CPD registration succesfully completed.');

%Find the correspondence, such that Y corresponds to X(C,:)
if corresp, C=cpd_Pcorrespondence(X,T,sigma2save,outliers); 
else C=0;
end;
end

