%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Download and Run extcmd
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, UC Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Traj_Download_Run(LTT_Data, si, Download_command, Run_command)

%% Download Traj 
if isequal(Download_command, 'Download')
    opt = {};
    opt.is_dual_arm = true;
    opt.LTT = 1;     % 0 - not in LTT mode; 1 - in LTT mode (default)
    opt.interpType = 'tcp';     % interpolation at: 'joint' space; 'TCP' space (default); 'no' interpolation
    opt.robot_idx = [1,2];
    opt.tcp = 0;        % 0 - joint position (default); 1 - tcp position
    opt.SI_jnt = 0;     % 0 - degree (default); 1 - radian
    opt.SI_tcp = 0;     % 0 - degree, mm (default); 1 - radian, m
    
    for i = 1:1
        extcmd = ExtCmdGenerate(LTT_Data.DesJntPos{i}, LTT_Data.ReplayTime{i}, si, si.ri{i}, LTT_Data.GrpCmd{i}, opt);
        ExtCmdDownload(si.ParamSgnID{i}, extcmd);
    end
end

%% start tp_prg movement
if isequal(Run_command, 'Run')
    opt = {};
    opt.is_dual_arm = true;
    opt.LTT = 1;     % 0 - not in LTT mode; 1 - in LTT mode (default)
    opt.interpType = 'tcp';     % interpolation at: 'joint' space; 'TCP' space (default); 'no' interpolation
    opt.robot_idx = [1,2];
    opt.tcp = 0;        % 0 - joint position (default); 1 - tcp position
    opt.SI_jnt = 0;     % 0 - degree (default); 1 - radian
    opt.SI_tcp = 0;     % 0 - degree, mm (default); 1 - radian, m
    
    IniJntPos = [LTT_Data.DesJntPos{1}(1,:); LTT_Data.DesJntPos{2}(1,:)];

    tp_pos_run(si, IniJntPos, opt);
    tp_prg_run(si, opt);
end
