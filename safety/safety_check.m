%% Wenjie Chen, 2016/07/15, FANUC Corporation
% safety check for single robot (boundary limit and self collision)
% Based on Yongxiang's work in 2015 summer internship

function IN_COL = safety_check(TransformationM, BASE_Z, RADIUS, boundary_min, boundary_max)

IN_COL = 0;

% first, check if the body is out of boundry. 
for i = 1:7
    p_i = TransformationM(1:3,4,i);
    if any(p_i < boundary_min | p_i > boundary_max)
        IN_COL = 10;
        return;
    end
end

%% checking body collision
p1 = TransformationM(1:3,4,1);
p2 = TransformationM(1:3,4,2);
p4 = TransformationM(1:3,4,4);
p7 = TransformationM(1:3,4,7);

% define cylinders 1 2 3 4 % hard code for LR mate 200iD
cylin1.p1 = [0;0;BASE_Z];
cylin1.p2 = [0;0;0];
cylin1.radius = RADIUS(1);
cylin2.p1 = p1;
cylin2.p2 = p2;
cylin2.radius = RADIUS(2);
cylin3.p1 = p2;
cylin3.p2 = p4;
cylin3.radius = RADIUS(3);
cylin4.p1 = p4;
cylin4.p2 = p7;
cylin4.radius = RADIUS(4);

if distance_measure(cylin1, cylin4, 'precise')
    IN_COL = 11;
    return;
end
if distance_measure(cylin2, cylin4, 'precise')
    IN_COL = 12;
    return;
end
if distance_measure(cylin1, cylin3, 'precise')
    IN_COL = 13;
    return;
end

end
