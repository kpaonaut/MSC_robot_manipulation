%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%  interpolate position reference and generate external command
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%  Input:
%  Xref  : [Nx6], robot joint position reference, N is the total horizon number.
%  robot : [1x2] cell, dual robot parameters
%  si    : [1x1] struct, simulation environment parameters
%  ri    : [1x1] struct, simulation robot parameters
%  dsrate: [1x1], downsample rate
% 
%  Output:
%  extcmd : [1x1] struct, similar to original extcmd structure.
%           Please see the output below
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function extcmd = Xref2ExtCmd(Xref, robot, si, ri, dsrate)

%% Interpolate reference 
nlink = robot.nlink;
horizon = size(Xref,1);
itp = si.itp;

 % Xref: [6 x N] [rad]
Vref = diff2(Xref)./(itp*dsrate);        % Vref: [6 x N] [rad/s]

% using "diff2" will fill 0 at the first time step, and leads to v = 0 at the first
% 2*dsrate step (but stable than using "diff")

Xnew = zeros(horizon*dsrate, nlink);
Vnew = zeros(horizon*dsrate, nlink);
tref = itp*dsrate:itp*dsrate:itp*dsrate*horizon;
tnew = itp:itp:itp*dsrate*horizon;

for i = 1:nlink
    Xnew(:,i) = interp1(tref, Xref(:,i), tnew, 'pchip');
end
for i = 1:nlink
    Vnew(:,i) = interp1(tref, Vref(:,i), tnew,'pchip');
end

%% Gerenate GrpCmd
grpcmd = [zeros(si.ext_vel_max_cnt,5), ones(si.ext_vel_max_cnt,1)];
totalsubmostep = 0;
for i = 1: length(robot.motion)
    for j = 1:length(robot.motion{i}.ReplayTime)
        submostep = length(robot.motion{i}.traj{j}.xref)/nlink;
        totalsubmostep = totalsubmostep + submostep;
        if robot.motion{i}.GrpCmd(j+1,4) ~=0
            actionperiord = totalsubmostep*dsrate:(totalsubmostep+submostep)*dsrate-1;
            grpcmd(actionperiord,:) = repmat(robot.motion{i}.GrpCmd(j+1,:),length(actionperiord),1);
        end
    end
end

%% Generate ExtCmd 
extcmd.vcmd_t = zeros(si.ext_vel_max_cnt,6);
extcmd.vcmd_EndCnt_t = size(Vnew,1);
motovel = (ri.interaction\Vnew')';
extcmd.vcmd_t(1:extcmd.vcmd_EndCnt_t,:) = motovel*(2^20/(2*pi)*si.itp); % Motor VelCmd [rad/s]
extcmd.inipos_t = Xnew(1,:).*180/pi;          % Joint Init Pos [deg]
extcmd.grippercmd = grpcmd;

extcmd.q = Xnew; % joint pos [rad]
extcmd.dq = Vnew; % joint vel [rad/s]
extcmd.t = tnew;