%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%       set up the linear constant constraint
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%  Based on Changliu's CFS algorithm 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function robot = costFn_Dual(xref, horizon, robot, varargin)
% When dualmotion = 'no', "robot" is moving robot
% When dualmotion = 'yes', "robot" is main robot to setup optimization

delta_t = 1;  % Vdiff and Adiff have some problem, currenrtly setting as 1
qd(1:robot.nlink,1:robot.nlink)=[4 0 0 0 0 0;
    0 3 0 0 0 0;
    0 0 3 0 0 0;
    0 0 0 2 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1];

w = [5;5;3;2;2;2];
% w = [10;5;3;2;1;1];
% w = [1;1;1;1;1;1];

if ~isempty(varargin) % Dual Robot
    robot2 = varargin{1};
    Nstate = robot.nlink+robot2.nlink;

    Q = [qd, zeros(robot.nlink); zeros(robot.nlink), qd];
    W = diag(repmat([w;w],horizon-2,1));
    
else                  % Single Robot
    Nstate = robot.nlink;
    Q = qd;
    W = diag(repmat(w,horizon-2,1));
end

Q1 = [];
for i=1:horizon
    Q1((i-1)*Nstate+1:i*Nstate,(i-1)*Nstate+1:i*Nstate)=Q*0.1;
    if i==horizon
        Q1((i-1)*Nstate+1:i*Nstate,(i-1)*Nstate+1:i*Nstate)=Q*10000;
    end
end

% The velocity
Vdiff = eye(horizon*Nstate) - diag(ones(1,(horizon-1)*Nstate),Nstate);
Vdiff = 1/delta_t*Vdiff;
Q2 = Vdiff(1:(horizon-1)*Nstate,:)'*Vdiff(1:(horizon-1)*Nstate,:);

W = 1*W;
% The acceleration
Adiff = Vdiff-diag(ones(1,(horizon-1)*Nstate),Nstate)+diag(ones(1,(horizon-2)*Nstate),Nstate*2);
Adiff = 1/delta_t^2*Adiff;
Q3 = Adiff(1:(horizon-2)*Nstate,:)'*W*Adiff(1:(horizon-2)*Nstate,:);

% The weight
% c = [1,10,20];
c = [1,10,20];

Qref = Q1*c(1)+Q2*c(2);
Qabs = Q3*c(3);


%% Extended State Cost

Qt = eye(horizon);

%% Extended input cost
Qu = blkdiag(eye((horizon-1)*Nstate), 5*eye(Nstate));
Qu =Qu;

%% Extended overall cost
Qe = blkdiag(10000*(Qref +Qabs), 0.001*Qt, 0*Qu);

%% 
robot.opt.H = Qref+Qabs;
robot.opt.f = -Qref*xref;
robot.Vdiff = Vdiff;
robot.Adiff = Adiff;


robot.opt.He = Qe;
robot.opt.fe = [-Qref*xref; zeros(horizon,1); zeros(horizon*Nstate,1)];
