%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Robot Experimentor
%       Function to Download and Run ExtCmd
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Te Tang, 09/28/2016       
%  MSC Lab, U.C.Berkeley
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function runExtCmd(ExtCmd, si)

for i = 1:2
    ExtCmdDownload(si.ParamSgnID{i}, ExtCmd{i});
end

opt = {};
opt.is_dual_arm = true;
opt.LTT = 1;     % 0 - not in LTT mode; 1 - in LTT mode (default)
opt.interpType = 'tcp';     % interpolation at: 'joint' space; 'TCP' space (default); 'no' interpolation
opt.robot_idx = [1,2];
opt.tcp = 0;        % 0 - joint position (default); 1 - tcp position
opt.SI_jnt = 0;     % 0 - degree (default); 1 - radian
opt.SI_tcp = 0;     % 0 - degree, mm (default); 1 - radian, m

IniJntPos = [ExtCmd{1}.inipos_t; ExtCmd{2}.inipos_t];
tp_pos_run(si, IniJntPos, opt);
tp_prg_run(si, opt);