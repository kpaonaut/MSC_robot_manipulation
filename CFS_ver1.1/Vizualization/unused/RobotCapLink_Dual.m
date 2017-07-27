function handle = RobotCapLink_Dual(robot, color, valpha)
DH=robot.DH;
offset = robot.base;
R_B2DS = robot.R_B2DS;
M={};
M{1}=[R_B2DS, offset;zeros(1,3) 1];
for i=1:6
    R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
        sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
        0  sin(DH(i,4)) cos(DH(i,4))];
    T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
    M{i+1}=M{i}*[R T;zeros(1,3) 1];
end

boundary=robot.boundary;
n=size(M,2);
for i=1:n %2:n-1
    if size(boundary{i},1)>0
        X=boundary{i}.X;
        Y=boundary{i}.Y;
        Z=boundary{i}.Z;
        kd=size(X,1);jd=size(X,2);
        for k=1:kd
            for j=1:jd
                newvec=[X(k,j),Y(k,j),Z(k,j)]*M{i}(1:3,1:3)'+M{i}(1:3,4)';
                X(k,j)=newvec(1);
                Y(k,j)=newvec(2);
                Z(k,j)=newvec(3);
            end
        end
        handle(i)=surf(X,Y,Z,'FaceColor',color,'EdgeColor','None');
        alpha(handle(i),valpha);
    end
end