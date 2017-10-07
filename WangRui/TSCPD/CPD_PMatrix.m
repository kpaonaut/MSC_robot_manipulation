%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Calculate the P Matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Changhao Wang, 09/14/2017
%  Modified by Rui Wang, 10/05/2017
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ P ] =CPD_PMatrix( G,X,Y,sigma,w,W,D )
% Parameter w (0 < w < 1) reflects our assumption on the amount of noise in
% the point set

% D represents the dimension of the point set
[N,~]=size(X);
[M,~]=size(Y);
P=zeros(M,N);
P_up=zeros(M,N);
P_down=zeros(M,N);
for m=1:M
    for n=1:N
        P_up(m,n)=exp(-1/(2*sigma^2)*norm(X(n,:)-(Y(m,:)+G(m,:)*W))^2);
        for k=1:M
            P_down(m,n)= P_down(m,n)+exp(-1/(2*sigma^2)*norm(X(n,:)-(Y(k,:)+G(k,:)*W))^2)+w/(1-w)*(2*pi*sigma^2)^(D/2)*M/N;
        end
%         if abs(P_up(m,n))<10^(-30)
%             P(m,n)=0;
%         else
            P(m,n)=P_up(m,n)/P_down(m,n);
      
    end
end

end

