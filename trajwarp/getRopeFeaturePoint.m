function rope_FeaturePoint = getRopeFeaturePoint(rope_LeftEndPoint, rope_RightEndPoint)

x_axis = rope_RightEndPoint - rope_LeftEndPoint;
x_axis = x_axis/norm(x_axis, 2);

cvx_begin
variables z_axis(3,1);
minimize(norm(z_axis - [0;0;1],2))     
subject to
x_axis'*z_axis == 0;
cvx_end

z_axis = z_axis/norm(z_axis,2);
y_axis = cross(z_axis, x_axis);

Rotation = [x_axis, y_axis, z_axis];
[U,~,V] = svd(Rotation);
Rotation = U*V';

T_left = [[Rotation, rope_LeftEndPoint];[0,0,0,1]];
T_right = [[Rotation, rope_RightEndPoint];[0,0,0,1]];

offset = 0.01;
rope_FeaturePoint = [getFeaturePointfromT(T_left, offset), getFeaturePointfromT(T_right, offset)];