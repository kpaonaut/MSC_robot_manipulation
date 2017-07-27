function [P1,P2] = find_clstdist_pos_consider_segment(C1,C2)
% find the position that achieves the closest distances. 
% coder.varsize('C1.p1',[3 1]);
% coder.varsize('C1.p2',[3 1]);
% coder.varsize('C2.p1',[3 1]);
% coder.varsize('C2.p2',[3 1]);
vec1 = (C1.p2-C1.p1);
vec2 = (C2.p2-C2.p1);
SAME_PLANE = ~any(vec2'*cross((C1.p1-C1.p2),(C2.p1-C1.p1))>0.02);

    P1 = zeros(3,1);
    P2 = zeros(3,1);
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
    P2 = [x2;y2;z2];
    % check if P2 is in the interval A and B


    if C2.p2(1)-C2.p1(1) ~=0
        in2 = (P2(1)-C2.p1(1))/(C2.p2(1)-C2.p1(1));
    elseif C2.p2(2)-C2.p1(2) ~=0
        in2 = (P2(2)-C2.p1(2))/(C2.p2(2)-C2.p1(2));
    else
        in2 = (P2(3)-C2.p1(3))/(C2.p2(3)-C2.p1(3));
    end
    



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
    % check if P1 is inside the interval
    if C1.p2(1)-C1.p1(1) ~=0
        in1 = (P1(1)-C1.p1(1))/(C1.p2(1)-C1.p1(1));
    elseif C1.p2(2)-C1.p1(2) ~=0
        in1 = (P1(2)-C1.p1(2))/(C1.p2(2)-C1.p1(2));
    else
        in1 = (P1(3)-C1.p1(3))/(C1.p2(3)-C1.p1(3));
    end

    if in1>=1
        P1 = C1.p2;
    elseif in1<=0
        P1 = C1.p1;
    end
    if in2 >=1
        P2 = C2.p2;
    elseif in2<=0
        P2 = C2.p1;
    end
if SAME_PLANE ==1 % do one more step, verify value

St = nan(7,4);
    t12 = (C1.p2 - C2.p1)'*vec2/norm(vec2).^2;
    % project C1.p2 into C2
    St(1:3,1) = C1.p2;
    if t12>=1
        St(4:6,1) = C2.p2;       
    elseif t12<=0
       St(4:6,1) = C2.p1;      
    else
        St(4:6,1) = C2.p1 + t12.*vec2;
    end
    St(7,1) = norm(St(1:3,1)-St(4:6,1));
    % project C1.p1 into C2
    t11 = (C1.p1 - C2.p1)'*vec2/norm(vec2).^2;
    St(1:3,2) = C1.p1;
    if t11>=1
        St(4:6,2) = C2.p2;       
    elseif t11<=0
       St(4:6,2) = C2.p1;      
    else
        St(4:6,2) = C2.p1 + t11.*vec2;
    end
    St(7,2) = norm(St(1:3,2)-St(4:6,2));
    % project C2.p2 into C1
    t21 = (C2.p2 - C1.p1)'*vec1/norm(vec1).^2;
    St(4:6,3) = C2.p2;
    if t21 >=1
        St(1:3,3) = C1.p2;
    elseif t21<=0
        St(1:3,3) = C1.p1;
    else
        St(1:3,3) = C1.p1 + t21.*vec1;
    end
    St(7,3) = norm(St(4:6,3)-St(1:3,3));
    % project C2.p1 into C1
    t22 = (C2.p1-C1.p1)'*vec1/norm(vec1).^2;
    St(4:6,4) = C2.p1;
    if t22>=1
        St(1:3,4) = C1.p2;
    elseif t22<=0
        St(1:3,4) = C1.p1;
    else
        St(1:3,4) = C1.p1 + t22.*vec1;
    end
   St(7,4) = norm(St(4:6,4)-St(1:3,4));
  % compare different norm
  index1 = find(St(7,:) == min(St(7,:)));
  P1 = St(1:3,index1(1));
  P2 = St(4:6,index1(1));
    
end
end













