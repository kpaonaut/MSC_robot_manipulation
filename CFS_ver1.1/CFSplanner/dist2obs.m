%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot CFS Simulator
%    calculate the robot arm to obstacle distance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Hsien-Chung Lin during FANUC internship in 2016
%  Based on Changliu's CFS algorithm 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Input:
%  RoPos   : [1xNlink cell], robot capsule postion
%  obs     : [1x1 cell], a block
% 
%  Output:
%  dist    : [1x1], minimum distance between robot and obstacle
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dist = dist2obs(RoPos, Obs)

if strcmp(Obs.type,'capsule')       % obs is a capsule
    dist = distLinSegMex(RoPos.p(:,1),RoPos.p(:,2), Obs.p(:,1),Obs.p(:,2)) - Obs.r;
    
elseif strcmp(Obs.type,'plane')     % obs is a plane
    dist = distL2P(RoPos.p(:,1),RoPos.p(:,2),Obs.p(:,1),3);
    
elseif strcmp(Obs.type,'sphere')    % obs is a sphere
    dist = distL2circ(RoPos.p(:,1),RoPos.p(:,2),Obs.p(:,1),Obs.r);
    
elseif strcmp(Obs.type, 'cylinder') % obs is a cylinder
    dist1 = distP2CylMex( Obs.p(:,1), Obs.p(:,2), Obs.r, RoPos.p(:,1));
    dist2 = distP2CylMex( Obs.p(:,1), Obs.p(:,2), Obs.r, RoPos.p(:,2));
    dist = min(dist1, dist2);

else                                % Default setting is capsule
    dist = distLinSegMex(RoPos.p(:,1),RoPos.p(:,2), Obs.p(:,1),Obs.p(:,2)) - Obs.r;
end

end
