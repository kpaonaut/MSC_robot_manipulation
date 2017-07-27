function [HuCap, HuPosVec]=HumanCap(M)

HM=cell(1,10);
for i=1:10
    HM{i}=eye(4);
end

HuCap={};
HuCap{1}.name = 'head';
HuCap{1}.p=[0 0;0 0;1.5 1.5]; %[0 0;0 0;1.5 1.5];
HuCap{1}.r=0.3;
HuCap{1}.type = 'capsule';

HuCap{2}.name = 'body';
HuCap{2}.p=[0 0;0 0;0.8 1.15];
HuCap{2}.r=0.22;
HuCap{2}.type = 'capsule';

t=[0;0;pi-0.3;pi-0.3;pi+0.3;pi+0.3;pi-0.2;pi-0.2;pi+0.2;pi+0.2];

%% Upper Arm
for i=3:2:5
    HuCap{i}.name = 'upper arm';
    HuCap{i}.r=0.07;
    HuCap{i}.p=[0 0;0 0;0 0.25];
    theta=t(i);
    R=[1 0 0; 0 cos(theta) -sin(theta);0 sin(theta) cos(theta)];
    T=[0;(i-4)*0.2;1.200];
    HM{i}=[R T;0 0 0 1];
    HuCap{i}.p=R*HuCap{i}.p+[T T];
    HuCap{i}.type = 'capsule';
end
%% Lower Arm
for i=4:2:6
    HuCap{i}.name = 'lower arm';
    HuCap{i}.p=[0 0;0 0;0 0.23];
    HuCap{i}.r=0.07;
    theta=t(i);
    R=[1 0 0; 0 cos(theta) -sin(theta);0 sin(theta) cos(theta)];
    T=[0;(i-5)*0.3;0.88];
    HM{i}=[R T;0 0 0 1];
    HuCap{i}.p=R*HuCap{i}.p+[T T];
    HuCap{i}.type = 'capsule';
end

%% Upper Leg
for i=7:2:9
    HuCap{i}.name = 'upper leg';
    HuCap{i}.r=0.1;
    HuCap{i}.p=[0 0;0 0;0 0.3];
    theta=t(i);
    R=[1 0 0; 0 cos(theta) -sin(theta);0 sin(theta) cos(theta)];
    T=[0;(i-8)*0.08;0.65];
    HM{i}=[R T;0 0 0 1];
    HuCap{i}.p=R*HuCap{i}.p+[T T];
    HuCap{i}.type = 'capsule';
end

%% Lower Leg
for i=8:2:10
    HuCap{i}.name = 'lower leg';
    HuCap{i}.r=0.08;
    HuCap{i}.p=[0 0;0 0;0 0.27];
    theta=t(i);
    R=[1 0 0; 0 cos(theta) -sin(theta);0 sin(theta) cos(theta)];
    T=[0;(i-9)*0.16;0.3];
    HM{i}=[R T;0 0 0 1];
    HuCap{i}.p=R*HuCap{i}.p+[T T];
    HuCap{i}.type = 'capsule';
end

xref=HuCap{1}.p(1,1); yref=HuCap{1}.p(2,1);

for i=1:10
    HuCap{i}.p=M(1:3,1:3)*(HuCap{i}.p-[xref xref;yref yref;0 0])+[M(1:3,4), M(1:3,4)];
end

HuPosVec = zeros(19,3);
HuPosVec(4,:) = HuCap{1}.p(:,1)';
HuPosVec(1,:) = HuCap{2}.p(:,2)';
HuPosVec(3,:) = HuCap{3}.p(:,1)';
HuPosVec(5,:) = HuCap{5}.p(:,1)';
HuPosVec(6,:) = HuCap{5}.p(:,2)';
HuPosVec(7,:) = HuCap{6}.p(:,2)';
HuPosVec(9,:) = HuCap{3}.p(:,1)';
HuPosVec(10,:) = HuCap{3}.p(:,2)';
HuPosVec(11,:) = HuCap{4}.p(:,2)';
HuPosVec(13,:) = HuCap{9}.p(:,1)';
HuPosVec(14,:) = HuCap{9}.p(:,2)';
HuPosVec(15,:) = HuCap{10}.p(:,2)';
HuPosVec(17,:) = HuCap{7}.p(:,1)';
HuPosVec(18,:) = HuCap{7}.p(:,2)';
HuPosVec(19,:) = HuCap{8}.p(:,2)';


end

