function val = cross_mul(u,v)
% make cross file by ourselves
% Made by Yongxiang Fan
a = (u(2)*v(3)-u(3)*v(2));
b = u(3)*v(1)-u(1)*v(3);
c = u(1)*v(2)-u(2)*v(1);
val = [a;b;c];
end