function PDF = GMR_MND(X,MU,Sigma_inv,Sigma_det)

d = numel(X);
PDF = 1/sqrt(abs(Sigma_det)*(2*pi)^d)*exp(-1/2*(X-MU)'*Sigma_inv*(X-MU));
