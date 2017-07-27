function dmin = Distance_TwoCylinders(cylin1,cylin2)
% input: parameters of two cylinders, cylin1 includes the coordinate of upper and lower
% centers, and the radius; MEASUREMENT has modes'rough' and 'precise' 
% output: minimum distance between two cylinders
crtcl_leng = (cylin1.radius+cylin2.radius);
vec1 = (cylin1.p2-cylin1.p1);
vec2 = (cylin2.p2-cylin2.p1);
% ua = vec1/norm(vec1);
% ub = vec2/norm(vec2);
% k = abs(ua'*ub);
% first, decide whether the two segments are in the same plane. 
SAME_PLANE = ~any(vec2'*cross((cylin1.p1-cylin1.p2),(cylin2.p1-cylin1.p1))>0.02);
if SAME_PLANE ==false
        dmin = distt_diff_plane(cylin1,cylin2);       
else
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
            Da2_b = norm(pb-pa2)- crtcl_leng;
        end
         % calculate Da1_b;
        tb = (pa1-pb1)'*vec2/norm(vec2)^2;
        if tb >1 ||tb<0
            Da1_b = inf;
        else
            pb = pb1 + tb*vec2;
            Da1_b = norm(pb-pa1)- crtcl_leng;
        end
        % calculate Db2_a;
        ta = (pb2-pa1)'*vec1/norm(vec1)^2;
        if ta >1 ||ta<0
            Db2_a = inf;
        else
            pa = pa1 + ta*vec1;
            Db2_a = norm(pa-pb2)- crtcl_leng;
        end
        % calculate Db1_a;
        ta = (pb1-pa1)'*vec1/norm(vec1)^2;
        if ta >1 ||ta<0
            Db1_a = inf;
        else
            pa = pa1 + ta*vec1;
            Db1_a = norm(pa-pb1)- crtcl_leng;
        end
        % calculate other possible minimum distances.% wrong here.
        Db1_a1 = norm(pb1-pa1)-crtcl_leng;
        Db1_a2 = norm(pb1-pa2)-crtcl_leng;
        Db2_a1 = norm(pb2-pa1)-crtcl_leng;
        Db2_a2 = norm(pb2-pa2)-crtcl_leng;
        % compare all the possible distances. 
        dmin = min([Da2_b,Da1_b,Db2_a,Db1_a,Db1_a1,Db1_a2,Db2_a1,Db2_a2]);
%         IN_COL = (dmin<0);
%     end
end

end
