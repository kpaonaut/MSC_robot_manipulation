%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Calculate the P Matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Rui Wang, 10/09/2017       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ P ] =CPD_PMatrix( X, Y, sigma2, w, D)
% The probability that X is an observation of Y with w the outlier weight
% Y is Z in my paper
% D is the dimension of the point set
[N, ~]=size(X);
[M, ~]=size(Y);
P=zeros(M + 1, N);
Ppre = zeros(N, M+1); % P_{i-1}
for m = 1 : M+1
    for n = 1 : N
        if m <= M
            Ppre(n, m) = 1/(2*pi*sigma2)^(D/2) * exp( - norm(X(n)-Y(m)) / (2*sigma2) );
        else
            Ppre(n, m) = 1/N;
        end
    end
for m = 1 : M + 1
    for n = 1 : N
        if m <= M
            num = Ppre(n, m) * (1-w)/M; % numerator
        else
            num = Ppre(n, m)*w;
        end
        den = sum(Ppre(n,1:M))*(1-w)/M + Ppre(n, M+1)*w;
        P(m, n) = num / den;
        if P(m, n) < 10e-10, P(m, n) = 0; end
    end
end
P = P(1:M, 1:N);
end
