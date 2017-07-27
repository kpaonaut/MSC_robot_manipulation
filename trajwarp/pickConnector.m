%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Motion for Picking Cable Connector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function pickConnector(imThOpt, opt, si)

ParamSgnID = si.ParamSgnID{opt.robot_idx};
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(ParamSgnID, 'off');

camData = cam2pcl(imThOpt);
region = [120, -250, 230; 370, 0, 280]/1000;  % specify the region to part of the upper stage
camData = camPCL2World(si, camData, region, opt);

% find the end point with the smallest y value
[~, min_idx] = min(camData.xyz_w(:,2));
% get surrounding point index whose distance is within 2cm 
endpoint_idx = (camData.xyz_w(:,1) - camData.xyz_w(min_idx,1)).^2 + (camData.xyz_w(:,2) - camData.xyz_w(min_idx,2)).^2  < 0.02^2;

% return error if cannot find connector in the specified region
xyz_m_w = mean(camData.xyz_w(endpoint_idx, :));
xyz_m_b = mean(camData.xyz_b(endpoint_idx, :));

if any(isnan(xyz_m_w)) || any(isnan(xyz_m_b)) || any(xyz_m_w < region(1,:)) || any(xyz_m_w > region(2,:)) 
    error('Cannot find cable connector in the specified region!!')
end

opt.tcp = 1;
PosCmd = [xyz_m_b * 1000, 180, 0, 219];
% For safety, currently hard code the table's height 
PosCmd(3) = -58;
UpPosCmd = PosCmd;
UpPosCmd(3) = UpPosCmd(3) + 210;

% Go and picking 
gripper_action(ParamSgnID, 'DRIVE', cellstr(repmat('fully open', numel(ParamSgnID), 1)));  
tp_pos_run(si, UpPosCmd, opt);
tp_pos_run(si, PosCmd, opt);
gripper_action(ParamSgnID, 'DRIVE', cellstr(repmat('grab cable taping tool firmly', numel(ParamSgnID), 1)));  
tp_pos_run(si, UpPosCmd, opt);

brake_on_off(ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);

end
