function [P1,P2] = find_clstdist_pos(C1,C2)
% find the position that achieves the closest distances. 
xa = C1.p1(1);
ya = C1.p1(2);
za = C1.p1(3);

xb = C1.p2(1);
yb = C1.p2(2);
zb = C1.p2(3);

xc = C2.p1(1);
yc = C2.p1(2);
zc = C2.p1(3);

xd = C2.p2(1);
yd = C2.p2(2);
zd = C2.p2(3);


H = xb-xa;
I = yb-ya;
J = zb-za;
K = xd-xc;
L = yd-yc;
M = zd-zc;
N = H*I*L - I^2*K - J^2*K + H*J*M;
O = H^2*L - H*I*K - I*J*M + J^2*L;
P = H*J*K - H^2*M - I^2*M + I*J*L;
Q = -xa*N + ya*O - za*P;
k = (O*yc - N*xc - P*zc - Q)/(N*K - O*L + P*M);
% position with the second cylinder. 
x2 = K*k + xc;
y2 = L*k + yc;
z2 = M*k + zc;
% find x1 y1 z1
N = L^2*H - L*I*K -M*J*K + M^2*H;
O = K*H*L - K^2*L -M^2*I + M*J*L;
P = K^2*J - K*H*M - L*I*M + L^2*J;
Q = -xc*N + yc*O - zc*P;
k = (O*ya - N*xa - P*za - Q)/(N*H - O*I + P*J);
x1 = H*k + xa;
y1 = I*k + ya;
z1 = J*k + za;
P1 = [x1;y1;z1];
P2 = [x2;y2;z2];


end














