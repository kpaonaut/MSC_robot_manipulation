function dis = distL2circ(point1, point2, circ, r)
    dis = min(  norm(point1 - circ, 2),  norm(point2-circ, 2) ) - r;
end