function [c,ceq] = nldisfn(x,DH,base,obs,cap,margin)
c = []; ceq = [];
nlink = size(DH,1);
[nobs,h] = size(obs);
for i=1:h
    xi = x((i-1)*nlink+1:i*nlink);
    for j=1:nobs
        I = margin - dist_arm_3D(xi,DH(1:nlink,:),base,obs{j,i},cap);
        c=[c;I];
    end
end
end