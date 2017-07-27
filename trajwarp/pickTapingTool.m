%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Motion for Picking Taping Tool
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function pickTapingTool(TapingTool_T_W, opt, si)

ParamSgnID = si.ParamSgnID{opt.robot_idx};
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(ParamSgnID, 'off');

Hole_T_W = TapingTool_T_W * transl(-0.047, 0, -0.037);       % marker - hole offset;
TCP_Hole_W =  Hole_T_W*[[rotx(pi),zeros(3,1)];[0,0,0,1]];    % flip the y,z direction of tcp

TCP_Hole_B = FrameTransform(TCP_Hole_W, 'T', 'W2B', si, opt.robot_idx);
TCP_Hole_Retreat_B =  TCP_Hole_B * transl(0,0,-0.1);  % (TCP -z direction)   
TCP_Hole_Above_B = transl(0,0,0.1) * TCP_Hole_B;

opt.SI_tcp = 0;   % 0 - degree, mm (default); 1 - radian, m  
TCP_Hole_B_xyzwpr = T2xyzwpr(TCP_Hole_B, opt);
TCP_Hole_Retreat_B_xyzwpr = T2xyzwpr(TCP_Hole_Retreat_B, opt);
TCP_Hole_Above_B_xyzwpr = T2xyzwpr(TCP_Hole_Above_B, opt);

% Go and picking 
opt.tcp = 1;
gripper_action(ParamSgnID, 'DRIVE', cellstr(repmat('fully open', numel(ParamSgnID), 1)));  
tp_pos_run(si, TCP_Hole_Retreat_B_xyzwpr, opt);
tp_pos_run(si, TCP_Hole_B_xyzwpr, opt);
gripper_action(ParamSgnID, 'DRIVE', cellstr(repmat('grab cable taping tool firmly', numel(ParamSgnID), 1)));  
tp_pos_run(si, TCP_Hole_Above_B_xyzwpr, opt);

brake_on_off(ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);

end