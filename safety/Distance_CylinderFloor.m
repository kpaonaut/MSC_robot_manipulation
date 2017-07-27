function dmin = Distance_CylinderFloor(Cylinder,BASE_Z)
d = min(Cylinder.p2(3),Cylinder.p1(3));
dmin = d - BASE_Z - Cylinder.radius;
end