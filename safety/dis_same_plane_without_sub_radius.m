function dmin = dis_same_plane_without_sub_radius(cylin1,cylin2)

vec1 = (cylin1.p2-cylin1.p1);
vec2 = (cylin2.p2-cylin2.p1);
pa1 = cylin1.p1;
pa2 = cylin1.p2;
pb1 = cylin2.p1;
pb2 = cylin2.p2;
% calculate Da2_b;
tb = (pa2-pb1)'*vec2/norm(vec2)^2;
if tb >1 ||tb<0
    Da2_b = inf;
else
    pb = pb1 + tb*vec2;
    Da2_b = norm(pb-pa2);
end
 % calculate Da1_b;
tb = (pa1-pb1)'*vec2/norm(vec2)^2;
if tb >1 ||tb<0
    Da1_b = inf;
else
    pb = pb1 + tb*vec2;
    Da1_b = norm(pb-pa1);
end
% calculate Db2_a;
ta = (pb2-pa1)'*vec1/norm(vec1)^2;
if ta >1 ||ta<0
    Db2_a = inf;
else
    pa = pa1 + ta*vec1;
    Db2_a = norm(pa-pb2);
end
% calculate Db1_a;
ta = (pb1-pa1)'*vec1/norm(vec1)^2;
if ta >1 ||ta<0
    Db1_a = inf;
else
    pa = pa1 + ta*vec1;
    Db1_a = norm(pa-pb1);
end
% calculate other possible minimum distances.% wrong here.
Db1_a1 = norm(pb1-pa1);
Db1_a2 = norm(pb1-pa2);
Db2_a1 = norm(pb2-pa1);
Db2_a2 = norm(pb2-pa2);
% compare all the possible distances. 
dmin = min([Da2_b,Da1_b,Db2_a,Db1_a,Db1_a1,Db1_a2,Db2_a1,Db2_a2]);
end