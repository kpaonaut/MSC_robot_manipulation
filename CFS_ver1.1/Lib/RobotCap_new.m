%% Draw boundary
% Unit: m
% r - radius of the capsule
% offset1, offset2 - two center line points on the Z axis
% xoffset, yoffset - offset in X and Y direction
% M - the translation matrices
% offset = [0,0,0.33];
% M={};
% M{1}=[eye(3) offset';zeros(1,3) 1];
% DH=robot.DH;
% for i=1:6
%     R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
%         sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
%         0  sin(DH(i,4)) cos(DH(i,4))];
%     T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
%     M{i+1}=M{i}*[R T;zeros(1,3) 1];
% end
% patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
hold on
boundary={}; % The handles;
%% base
valpha=0.3; r=0.13; offset1=-0.33;offset2=0.25-0.33;
[X1,Y1,Z1]=sphere;
nd=size(X1,1);md=(nd+1)/2;
X=[X1(md,:).*r;X1(md:nd,:).*r];
Y=[Y1(md,:).*r;Y1(md:nd,:).*r];
Z=[Z1(md,:).*r+offset1;Z1(md:nd,:).*r+offset2];
i=1;
boundary{i}.X=X;boundary{i}.Y=Y;boundary{i}.Z=Z;
for k=1:md
for j=1:nd
    newvec=[X(k,j),Y(k,j),Z(k,j)]*M{1}(1:3,1:3)'+M{1}(1:3,4)'+offset;
    X(k,j)=newvec(1);
    Y(k,j)=newvec(2);
    Z(k,j)=newvec(3);
end
end
boundary{i}.handle=surf(X,Y,Z); 
alpha(boundary{i}.handle,valpha);
%% link1
% r=0.10;offset1=-0.07;offset2=0.07;xoffset=-0.03;yoffset=-0.05
% [X,Y,Z]=sphere;
% X=X.*r+xoffset;
% Y=Y.*r+yoffset;
% Z=Z.*r;
% nd=size(Z,1);md=(nd+1)/2;
% Z(1:md-1,:)=Z(1:md-1,:)+offset1;
% Z(md:nd,:)=Z(md:nd,:)+offset2;
% i=2;
% boundary{i}.X=X;boundary{i}.Y=Y;boundary{i}.Z=Z;
% for k=1:nd
% for j=1:nd
%     newvec=[X(k,j),Y(k,j),Z(k,j)]*M{2}(1:3,1:3)'+M{2}(1:3,4)';
%     X(k,j)=newvec(1);
%     Y(k,j)=newvec(2);
%     Z(k,j)=newvec(3);
% end
% end
% boundary{i}.handle=surf(X,Y,Z);
% alpha(boundary{i}.handle,valpha);
%% link2
r=0.13;offset1=-0.40;offset2=0;
[X1,Y1,Z1]=sphere;
nd=size(X1,1);md=(nd+1)/2;
X(1:md-1,:)=Z1(1:md-1,:).*r+offset1;
X(md:nd,:)=Z1(md:nd,:).*r+offset2;
Y=X1.*r;
Z=Y1.*r;
i=3;
boundary{i}.X=X;boundary{i}.Y=Y;boundary{i}.Z=Z;
for k=1:nd
for j=1:nd
    newvec=[X(k,j),Y(k,j),Z(k,j)]*M{3}(1:3,1:3)'+M{3}(1:3,4)'+offset;
    X(k,j)=newvec(1);
    Y(k,j)=newvec(2);
    Z(k,j)=newvec(3);
end
end
boundary{i}.handle=surf(X,Y,Z);
alpha(boundary{i}.handle,valpha);

% r=0.13;offset1=-0.36;offset2=0;
% [X1,Y1,Z1]=sphere;
% nd=size(X1,1);md=(nd+1)/2;
% X(1:md-1,:)=Z1(1:md-1,:).*r+offset1;
% X(md:nd,:)=Z1(md:nd,:).*r+offset2;
% Y=X1.*r;
% Z=Y1.*r;
% i=3;
% boundary{i}.X=X;boundary{i}.Y=Y;boundary{i}.Z=Z;
% for k=1:nd
% for j=1:nd
%     newvec=[X(k,j),Y(k,j),Z(k,j)]*M{3}(1:3,1:3)'+M{3}(1:3,4)';
%     X(k,j)=newvec(1);
%     Y(k,j)=newvec(2);
%     Z(k,j)=newvec(3);
% end
% end
% boundary{i}.handle=surf(X,Y,Z);
% alpha(boundary{i}.handle,valpha);
%% link3
% r=0.220;offset1=0;offset2=0;
% [X,Y,Z]=sphere;
% X=X.*r;
% Y=Y.*r;
% Z=Z.*r;
% nd=size(Z,1);md=(nd+1)/2;
% Z(1:md-1,:)=Z(1:md-1,:)+offset1;
% Z(md:nd,:)=Z(md:nd,:)+offset2;
% i=4;
% boundary{i}.X=X;boundary{i}.Y=Y;boundary{i}.Z=Z;
% for k=1:nd
% for j=1:nd
%     newvec=[X(k,j),Y(k,j),Z(k,j)]*M{4}(1:3,1:3)'+M{4}(1:3,4)';
%     X(k,j)=newvec(1);
%     Y(k,j)=newvec(2);
%     Z(k,j)=newvec(3);
% end
% end
% boundary{i}.handle=surf(X,Y,Z);
% alpha(boundary{i}.handle,valpha);
%% link4
r=0.08;offset1=0;offset2=0.40;
[X1,Y1,Z1]=sphere;
X=X1.*r;
Y=Z1.*r;
Z=Y1.*r;
nd=size(Z,1);md=(nd+1)/2;
Y(1:md-1,:)=Y(1:md-1,:)+offset1;
Y(md:nd,:)=Y(md:nd,:)+offset2;
i=5;
boundary{i}.X=X;boundary{i}.Y=Y;boundary{i}.Z=Z;
for k=1:nd
for j=1:nd
    newvec=[X(k,j),Y(k,j),Z(k,j)]*M{5}(1:3,1:3)'+M{5}(1:3,4)'+offset;
    X(k,j)=newvec(1);
    Y(k,j)=newvec(2);
    Z(k,j)=newvec(3);
end
end
boundary{i}.handle=surf(X,Y,Z);
alpha(boundary{i}.handle,valpha);

%% link5
r=0.070;offset1=-0.30;offset2=0.000;
[X,Y,Z]=sphere;
X=X.*r;
Y=Y.*r;
Z=Z.*r;
nd=size(Z,1);md=(nd+1)/2;
Z(1:md-1,:)=Z(1:md-1,:)+offset1;
Z(md:nd,:)=Z(md:nd,:)+offset2;
i=6;
boundary{i}.X=X;boundary{i}.Y=Y;boundary{i}.Z=Z;
for k=1:nd
for j=1:nd
    newvec=[X(k,j),Y(k,j),Z(k,j)]*M{6}(1:3,1:3)'+M{6}(1:3,4)'+offset;
    X(k,j)=newvec(1);
    Y(k,j)=newvec(2);
    Z(k,j)=newvec(3);
end
end
boundary{i}.handle=surf(X,Y,Z);
alpha(boundary{i}.handle,valpha);

% r=0.090;offset1=-0.30;offset2=0.0100;
% [X,Y,Z]=sphere;
% X=X.*r;
% Y=Y.*r;
% Z=Z.*r;
% nd=size(Z,1);md=(nd+1)/2;
% Z(1:md-1,:)=Z(1:md-1,:)+offset1;
% Z(md:nd,:)=Z(md:nd,:)+offset2;
% i=6;
% boundary{i}.X=X;boundary{i}.Y=Y;boundary{i}.Z=Z;
% for k=1:nd
% for j=1:nd
%     newvec=[X(k,j),Y(k,j),Z(k,j)]*M{6}(1:3,1:3)'+M{6}(1:3,4)';
%     X(k,j)=newvec(1);
%     Y(k,j)=newvec(2);
%     Z(k,j)=newvec(3);
% end
% end
% boundary{i}.handle=surf(X,Y,Z);
% alpha(boundary{i}.handle,valpha);
%% link6
r=0.060;offset1=0.050;offset2=0.180;
[X1,Y1,Z1]=sphere;
X=Z1.*r;
Y=Y1.*r;
Z=X1.*r+0.1107;
nd=size(Z,1);md=(nd+1)/2;
X(1:md-1,:)=X(1:md-1,:)+offset1;
X(md:nd,:)=X(md:nd,:)+offset2;
i=7;
boundary{i}.X=X;boundary{i}.Y=Y;boundary{i}.Z=Z;
for k=1:nd
for j=1:nd
    newvec=[X(k,j),Y(k,j),Z(k,j)]*M{i}(1:3,1:3)'+M{i}(1:3,4)'+offset;
    X(k,j)=newvec(1);
    Y(k,j)=newvec(2);
    Z(k,j)=newvec(3);
end
end
boundary{i}.handle=surf(X,Y,Z);
alpha(boundary{i}.handle,valpha);

%% Setup the background
xlim = [-200, 1000]/1000;
ylim = [-500, 500]/1000;
zlim = [-330, 1200]/1000;

axis equal
axis([-.200 1.000, -.500 .500,-.500,1.000])
axis([xlim, ylim, zlim])
view([1,-0.5,0.4])
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); 
% lighting flat
% light=camlight('left');

wall{1}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(1),ylim(2),ylim(2),ylim(1)],[zlim(1),zlim(1),zlim(1),zlim(1)],[0.1,0.1,0.1]);
wall{2}.handle=fill3([xlim(1),xlim(1),xlim(1),xlim(1)],[ylim(1),ylim(1),ylim(2),ylim(2)],[zlim(1),zlim(2),zlim(2),zlim(1)],[0,0.9,0.9]);
wall{3}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(2),ylim(2),ylim(2),ylim(2)],[zlim(1),zlim(2),zlim(2),zlim(1)],[0,0.9,0.9]);



% zlim=1;
% xlim=[-1,1];
% ylim=[-1,1];
% view([1,-0.5,0.4])
% axis equal
% axis([xlim,ylim,0,zlim])
% lighting=camlight('left')
% %lighting phong
% set(gca,'Color',[0.8 0.8 0.8]);
% %set(gcf,'Renderer','zbuffer');
% 
% wall{1}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(1),ylim(2),ylim(2),ylim(1)],[0,0,0,0],[0.1,0.1,0.1]);
% wall{2}.handle=fill3([xlim(1),xlim(1),xlim(1),xlim(1)],[ylim(1),ylim(1),ylim(2),ylim(2)],[0,zlim,zlim,0],[0,0.9,0.9]);
% wall{3}.handle=fill3([xlim(1),xlim(1),xlim(2),xlim(2)],[ylim(2),ylim(2),ylim(2),ylim(2)],[0,zlim,zlim,0],[0,0.9,0.9]);


