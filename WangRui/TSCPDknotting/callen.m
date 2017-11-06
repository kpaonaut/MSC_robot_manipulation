function len = callen(x, y)
n = size(x, 1);
s = 0;
for i = 1:n-1
    s = s + sqrt((x(i)-x(i+1))^2+(y(i)-y(i+1))^2);
end
len = s/(n-1);
end