function phi = nlcon(x,DH,base,obs,cap,margin)

nlink = size(DH,1);
[nobs,h] = size(obs);
phi = zeros(h*nobs,1);
for i=1:h
    xi = x((i-1)*nlink+1:i*nlink);
    for j=1:nobs
        I = dist_arm_3D(xi,DH(1:nlink,:),base,obs{j,i},cap) - margin;
        phi((i-1)*nobs+j)=I;
    end
end
end