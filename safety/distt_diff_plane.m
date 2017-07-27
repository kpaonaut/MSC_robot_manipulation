function dmin = distt_diff_plane(C1,C2)
% calculate the distance between two lines: (p11,p12) v.s. (p21,p22)
% vec1 = C1.p1-C1.p2;
% vec2 = C2.p1-C2.p2;
% mu_vec = cross_mul(vec1,vec2); % cross product 
% mu_normal = mu_vec/norm(mu_vec);% normal cross product 
% normal_vec = ((C1.p1-C2.p1)'*mu_normal); % absolute distance
[P1,P2] = find_clstdist_pos(C1,C2);
normal_vec = P2-P1;
% whether P1 or P2 are inside the interval
if C1.p2(1)-C1.p1(1) ~=0
    in1 = (P1(1)-C1.p1(1))/(C1.p2(1)-C1.p1(1));
elseif C1.p2(2)-C1.p1(2) ~=0
    in1 = (P1(2)-C1.p1(2))/(C1.p2(2)-C1.p1(2));
else
    in1 = (P1(3)-C1.p1(3))/(C1.p2(3)-C1.p1(3));
end

if C2.p2(1)-C2.p1(1) ~=0
    in2 = (P2(1)-C2.p1(1))/(C2.p2(1)-C2.p1(1));
elseif C2.p2(2)-C2.p1(2) ~=0
    in2 = (P2(2)-C2.p1(2))/(C2.p2(2)-C2.p1(2));
else
    in2 = (P2(3)-C2.p1(3))/(C2.p2(3)-C2.p1(3));
end

if in1<=1 && in1>=0 && in2 <=1 && in2>=0% both are inside interval, distance is abs(normal_vector)
    d = norm(normal_vec);
else % at least one is not inside the interval
    cylin1.p1 = C1.p1 + normal_vec;
    cylin1.p2 = C1.p2 + normal_vec;
    cylin1.radius = C1.radius;
    d_same = dis_same_plane_without_sub_radius(cylin1,C2);
    d = sqrt(norm(normal_vec).^2 + d_same.^2);
end
dmin = d - C1.radius - C2.radius;
end