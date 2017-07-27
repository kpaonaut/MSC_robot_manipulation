% Test Trajecrory Generation
% a Trajectroy Generation test code made by Hsien-Chung Lin


% qr1 = [69,0,0,0, -90,0];
% qr2 = [-69,30,0,0, -90,0]; % deg
% qr3 = [69,0,0,0, -90,0];
% qr4 = [69,0,0,0, 90,0];
%  TimeVec = [5, 2, 2]; % Time interval between two pos, should be size(qr,2)-1
 
% Robot No.1 LTT1
qr1 = [35.6743    2.7470    7.7005   -1.5183  -92.3106   90.0195];
qr2 = [ 6.5575    4.5639   -4.4206   27.3985  -64.3502  109.1018];
qr3 = [ 2.3919    8.7251   -9.2963   28.2294  -53.7120  105.2907];
qr4 = [-5.4480   17.1914    0.8988   -0.1213  -70.6822  132.9203];
qr5 = [-2.8703   14.3313   -3.2206    0.1054  -69.1968  130.3371];
qr = [qr1; qr2; qr3; qr4; qr5];
% 
TimeVec = [2, 1, 1, 1]; % Time interval between two pos, should be size(qr,2)-1

% Robot No.2 LTT1
% qr1 = [-14.8707   16.3955   13.5122    1.3748  -87.6183 -127.0817];
% qr2 = [-29.7067    5.4803    0.6395    1.5160  -85.0276 -111.5128];
% qr3 = [-39.0149    3.2915  -13.2809  -15.9429  -70.4909  -99.2033 ];
% qr4 = [-46.2656    7.1513  -15.4270  -31.8481  -72.7058  -84.2751];
% qr5 = [-59.5382    6.2631   13.1875  -28.8509 -103.8313  -85.5495];
% qr6 = [-37.6929    2.6527   13.8643    3.1243  -97.1108 -102.7185];
% qr = [qr1; qr2; qr3; qr4; qr5; qr6];

% TimeVec = [1, 1, 1, 1, 1];
%%
% q = mstraj(qr(2:end,:),[],TimeVec,qr(1,:),0.001,1,'verbose');



%%

InitSim; % acquire si and ri parameters

%%
% Some useful option
opt.interpType = 'linear';  % 'joint', 'TCP', 'linear'
opt.SI_jnt = 0; % 0 - degree (default); 1 - radian
opt.mstraj =0;
opt.TaccRatio = diag([0.75,0.6,0.75]);
extcmd = ExtCmdGenerate(qr, TimeVec, si, ri{1}, [], opt);

t = extcmd.t; % [sec]
q = deg2rad(extcmd.q); % [rad]
dq = extcmd.dq; % [rad/s]

figure; 
subplot(2,1,1); plot(t,q)
subplot(2,1,2); plot(t,dq)
% 
xref = downsample(q,20)'; % sampling time is 0.16


%%
xref =xref(:);
save('LRmate200R1Traj_LTT3.mat','xref')