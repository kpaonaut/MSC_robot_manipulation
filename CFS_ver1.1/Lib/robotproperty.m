function robot=robotproperty(id)
robot.name = id;
switch id
%% LR Mate 200iD
    case 'LRMate200iD'
        %the constants
        robot.nlink=6;
        robot.umax=10;
        robot.margin=3;
        robot.delta_t=0.05;
        robot.thetamax=[-pi,pi;0,pi;-pi,pi;-pi,pi;-pi/2,pi/2;-pi,pi];
        robot.thetadotmax=[1;1;1;1;0.5;0.5];
        %The length of each links and DH parameter and base
        robot.l=[2;6;5];
        robot.DH=[0,0,0.050,-1.5708;
                  -1.5708,0,0.330,3.1416;
                  0,0,0.035,-1.5708;
                  0,-0.335,0,1.5708;
                  0,0,0,-1.5708;
                  0,-0.080,0,3.1416];%theta,d,a,alpha
        robot.base=[0;0;0.329];%origin
        robot.cap={};
        robot.cap{1}.p=[0 0;0 0;0 0];
        robot.cap{1}.r=0;
        
        robot.cap{2}.p=[-0.36 0;0 0;0 0];
        robot.cap{2}.r=0.13;
        
        robot.cap{3}.p=[-0.03 -0.03;0 0;0.05 0.05];
        robot.cap{3}.r=0;
        
        robot.cap{4}.p=[0 0;0 0.35;0 0];
        robot.cap{4}.r=0.08;
        
        robot.cap{5}.p=[0 0;0 0;-0.3 0.010];
        robot.cap{5}.r=0.09;
        
        robot.cap{6}.p=[0.05 0.17;0 0; 0.135 0.135];
        robot.cap{6}.r=0.06;
        
        
        load([pwd,'/figure/LRMate200iDCapsules.mat']);
        robot.boundary=RoBoundary;

%% LR Mate 200iD7L Robot No.1        
    case 'LRMate200iD7L'
        %the constants
        robot.nlink=6;
        robot.umax=10;
        robot.margin=0.05;
        robot.delta_t=0.16; %0.05
        robot.thetamax = [-170, -100, -72, -190, -125, -360;
                           170,  145,  240, 190, 125, 360]'.*pi/180;
        robot.thetadotmax = [35,  35,  50,  50,  50,  50]'.*pi/180;
        
        %The length of each links and DH parameter and base
%         robot.l=[0.65;0.77;0.1;0.74;0;0.1]; Not used in CFS
        robot.DH=[0,0,0.050,-1.5708;
                  -1.5708,0,0.440,3.1416;
                  0,0,0.035,-1.5708;
                  0,-0.420,0,1.5708;
                  0,0,0,-1.5708;
                  0,-0.080,0,3.1416];%theta,d,a,alpha
        robot.base = [0.0825   -0.0825    0.3300]';  %Berkeley
%         robot.base = [0;0;0.330]; % FANUC
        robot.R_B2DS = rpy2r([0,0,0]);
        robot.cap={};
        robot.cap{1}.p=[0 0;0 0;0 0];
        robot.cap{1}.r=0;
        
        robot.cap{2}.p=[-0.4 0;0 0;0 0];
        robot.cap{2}.r=0.13;
        
        robot.cap{3}.p=[-0.03 -0.03;0 0;0.05 0.05];
        robot.cap{3}.r=0;
        
        robot.cap{4}.p=[0 0;0 0.4;0 0];
        robot.cap{4}.r=0.068;
        
        robot.cap{5}.p=[0 0;0 0;-0.26 0.01];
        robot.cap{5}.r=0.01;
        
        robot.cap{6}.p=[0.05, 0.18;0 0; 0.1107 0.1107];
        robot.cap{6}.r=0.06;
        
        load('figure/LRMate200iD7LCapsules.mat');
        robot.boundary = RoBoundary;
%% LR Mate 200iD7L Robot No.2
    case 'LRMate200iD7Ld2'
        %the constants
        robot.name = 'LRMate200iD7L'; % still using LRMate200id7L 
        robot.nlink=6;
        robot.umax=10;
        robot.margin=0.05;
        robot.delta_t=0.16; %0.05
        robot.thetamax = [-170, -100, -72, -190, -125, -360;
                           170,  145,  240, 190, 125, 360]'.*pi/180;
        robot.thetadotmax = [35,  35,  50,  50,  50,  50]'.*pi/180;
        
        %The length of each links and DH parameter and base
%         robot.l=[0.65;0.77;0.1;0.74;0;0.1]; Not used in CFS
        robot.DH=[0,0,0.050,-1.5708;
                  -1.5708,0,0.440,3.1416;
                  0,0,0.035,-1.5708;
                  0,-0.420,0,1.5708;
                  0,0,0,-1.5708;
                  0,-0.080,0,3.1416];%theta,d,a,alpha
%         robot.base=[0.5825   -0.7825    0.3300]'; %[0;0;0.330];%origin %    FANUC
        robot.base=[0.0825   -1.3515    0.3300]'; % Berkeley
%         robot.R_B2DS = rpy2r([pi/2, 0, 0], 'zyx');  % FANUC
        robot.R_B2DS = rpy2r([0,0,0]); % Berkeley
        robot.cap={};
        robot.cap{1}.p=[0 0;0 0;0 0];
        robot.cap{1}.r=0;
        
        robot.cap{2}.p=[-0.4 0;0 0;0 0];
        robot.cap{2}.r=0.13;
        
        robot.cap{3}.p=[-0.03 -0.03;0 0;0.05 0.05];
        robot.cap{3}.r=0;
        
        robot.cap{4}.p=[0 0;0.0 0.4;0 0];
        robot.cap{4}.r=0.068;
        
        robot.cap{5}.p=[0 0;0 0;-0.26 0.01];
        robot.cap{5}.r=0.01;
        
        robot.cap{6}.p=[0.05, 0.18;0 0; 0.1107 0.1107];
        robot.cap{6}.r=0.06;
        
        load('figure/LRMate200iD7LCapsules.mat');
        robot.boundary = RoBoundary;

%% M16iB        
    case 'M16iB'
        %the constants
        robot.nlink=6;
        robot.umax=10;
        robot.thetamax=[-pi,pi;0,pi;-pi,pi;-pi,pi;-pi/2,pi/2;-pi,pi];
        robot.thetadotmax=[1;1;1;1;0.5;0.5];
        robot.margin=0.5;
        robot.delta_t=0.5;
        %The length of each links and DH parameter and base
        robot.l=[0.65;0.77;0.1;0.74;0;0.1];
        robot.DH=[0.5, 0.65, 0.15, 1.5708;
            1.5708, 0, 0.77, 0;
            0, 0, 0.1, 1.5708;
            0, 0.74, 0, -1.5708;
            -pi/2, 0, 0, 1.5708;
            pi, 0.1, 0, 0];%theta,d,a,alpha
        robot.base=[0;0;0];%origin
        
        robot.cap={};
        robot.cap{1}.p=[0 0;0 0;-0.1 0.1];
        robot.cap{1}.r=0.15;
        
        robot.cap{2}.p=[-0.75 0;0 0;-0.15 -0.15];
        robot.cap{2}.r=0.13;
        
        robot.cap{3}.p=[-0.03 -0.03;0 0;0.05 0.05];
        robot.cap{3}.r=0.22;
        
        robot.cap{4}.p=[0 0;0 0.55;0 0];
        robot.cap{4}.r=0.11;
        
        robot.cap{5}.p=[0 0;0 0;-0.05 0.110];
        robot.cap{5}.r=0.07;
        
        robot.cap{6}.p=[-0.11 -0.11;0 0; 0.09 0.09];
        robot.cap{6}.r=0.11;
        
        load('figure/M16iBCapsules.mat');
        robot.boundary=RoBoundary;
        
       
end

%The kinematic matrices
robot.A=[eye(robot.nlink) robot.delta_t*eye(robot.nlink);
        zeros(robot.nlink) eye(robot.nlink)];
robot.B=[0.5*robot.delta_t^2*eye(robot.nlink);
        robot.delta_t*eye(robot.nlink)];
robot.Ac=[eye(3) robot.delta_t*eye(3);
        zeros(3) eye(3)];
robot.Bc=[0.5*robot.delta_t^2*eye(3);
        robot.delta_t*eye(3)];
robot.C=[];
robot.D=[];
robot.Q=[];
robot.R=[];

robot.x(1:robot.nlink*2,1)=[robot.DH(:,1);zeros(robot.nlink,1)];%(theta1,theta2,theta,3,theta1dot,theta2dot,theta3dot)
robot.pos=CapPos(robot.base,robot.DH,robot.cap);
robot.wx(1:3,1)=robot.pos{robot.nlink}.p(:,2);
robot.wx(4:6,1)=[0;0;0];%endpoint state(x4,y4,z4,x4dot,y4dot,z4dot)
robot.mx=robot.wx;%closest point state

robot.ref.x=robot.x;
robot.innoise=0;
robot.outnoiseself=0;
robot.outnoisestar=0;
robot.obs.xself=[];
robot.obs.xstar=[];
robot.obs.goal=[];

robot.obs.A=robot.A;
robot.obs.B=robot.B;
robot.obs.C=robot.C;
robot.obs.D=robot.D;
robot.obs.Q=robot.Q;
robot.obs.R=robot.R;
robot.score=0;
robot.flag=0;


%For SSA
robot.const.P1=[eye(robot.nlink) zeros(robot.nlink);zeros(robot.nlink) zeros(robot.nlink)];
robot.const.P2=[zeros(robot.nlink) 0.5*eye(robot.nlink);0.5*eye(robot.nlink) zeros(robot.nlink)];
robot.const.P3=[zeros(robot.nlink) zeros(robot.nlink);zeros(robot.nlink) eye(robot.nlink)];

%For collision check
robot.profile={};

%For Optimization

end


