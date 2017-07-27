%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       *Test File* for testing 3D Mouse
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Mouse3D('start');
while 1
    % max robot speed in m/s and the rotation matrix to match with tool frame
    %     Scale = 0.2 * [0 0 1; -1 0 0; 0 -1 0];    % mouse held at hand
    Scale = 0.2 * [0 1 0; 1 0 0; 0 0 -1];     % mouse mounted on robot
    out = Mouse3D('get');
    Speed = [Scale * out.pos' / 2500; Scale * out.ang * out.rot' / 2500 * 2]'
    pause(0.1)
end
                
Mouse3D('stop');