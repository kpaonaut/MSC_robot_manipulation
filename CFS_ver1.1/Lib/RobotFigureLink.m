%% LR Mate 200iD7L
ROBOT = 'LRMate200iD7L';
load('figure/LRMate200iD7L.mat');
robot = robotproperty(ROBOT);

offset = [0,0,0.330];
DH=robot.DH;

figure; hold on

M={};
M{1}=eye(4);
% M{1}(3,1:3) = offset';
for i=1:6
    R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
        sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
        0  sin(DH(i,4)) cos(DH(i,4))];
    T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
    M{i+1}=M{i}*[R T;zeros(1,3) 1];
end
valpha = 0.5;


f=base{1}.f; v=base{1}.v+ones(size(base{1}.v,1),1)*offset; c=base{1}.c; color=base{1}.color;
patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');


for i=1:6
    v=link{i}.v; f=link{i}.f; c=link{i}.c; color=link{i}.color;
    for j=1:size(v,1)
        v(j,:)=v(j,:)*M{i+1}(1:3,1:3)'+M{i+1}(1:3,4)'+offset;
    end
    gcf;
    h=patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
    alpha(h,valpha);   
end

i=6;
v=payload{1}.v;f=payload{1}.f;c=payload{1}.c;color=payload{1}.color;
for j=1:size(v,1)
    v(j,:)=v(j,:)*M{i+1}(1:3,1:3)'+M{i+1}(1:3,4)'+offset;
end
patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');    
% 

xlim = [-200, 1200]/1000;
ylim = [-500, 500]/1000;
zlim = [-330, 1200]/1000;

axis equal
axis([-.200 1.000, -.500 .500,-.500,1.000])
axis([xlim, ylim, zlim])
view([1,-0.5,0.4])
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); 
lighting flat
light=camlight('left');


%% M16iB

% load('figure/M16iB-figure.mat');
% DH=robot.DH;
% M={};
% M{1}=eye(4);
% for i=1:6
%     R=[cos(DH(i,1)) -sin(DH(i,1))*cos(DH(i,4)) sin(DH(i,1))*sin(DH(i,4));
%         sin(DH(i,1)) cos(DH(i,1))*cos(DH(i,4)) -cos(DH(i,1))*sin(DH(i,4));
%         0  sin(DH(i,4)) cos(DH(i,4))];
%     T=[DH(i,3)*cos(DH(i,1));DH(i,3)*sin(DH(i,1));DH(i,2)];
%     M{i+1}=M{i}*[R T;zeros(1,3) 1];
% end
% 
% for i=1:6
%     v=link{i}.v; f=link{i}.f; c=link{i}.c; color=link{i}.color;
%     for j=1:size(v,1)
%         v(j,:)=v(j,:)*M{i+1}(1:3,1:3)'+M{i+1}(1:3,4)'.*1000+offset;
%     end
%     gcf;
%     h=patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
%     alpha(h,valpha);
%     if i==1 || i==3
%         for k=1:4
%             v=link{i}.motor{k}.v; f=link{i}.motor{k}.f; c=link{i}.motor{k}.c; color=link{i}.motor{k}.color;
%             for j=1:size(v,1)
%                 v(j,:)=v(j,:)*M{i+1}(1:3,1:3)'+M{i+1}(1:3,4)'.*1000+offset;
%             end
%             h=patch('Faces',f,'Vertices',v,'FaceVertexCData',c,'FaceColor',color,'EdgeColor','None');
%             alpha(h,valpha);
%         end
%     end
%     
% end