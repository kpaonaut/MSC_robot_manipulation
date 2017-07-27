function dis = distL2P(point1, point2, plane, i)
    dis = min(  abs(point1(i) - plane(i)),  abs(point2(i)-plane(i))  );
end