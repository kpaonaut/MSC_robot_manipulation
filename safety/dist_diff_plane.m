function d = dist_diff_plane(p11,p12,p21,p22)
vec1 = p11-p12;
vec2 = p21-p22;
mu_vec = cross(vec1,vec2);
mu_normal = mu_vec/norm(mu_vec);
d = abs((p11-p21)'*mu_normal);
end