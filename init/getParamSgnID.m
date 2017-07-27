%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       Setup parameter id for block parameter tuning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/06/24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ParamSgnID = getParamSgnID(si)

tg = SimulinkRealTime.target;
ParamSgnID = cell(si.robot_num,1);

for rn = 1:si.robot_num
    ParamSgnID{rn}.robot_no = rn;
    ParamSgnID{rn}.timeGripperInputHold = si.timeGripperInputHold;
    
    %% System state related
    block_path = ['Scopes/SysState ', num2str(rn), '/'];
    ParamSgnID{rn}.EtherCATOpState = getsignalid(tg, [block_path 'Gain_EtherCATOpState']);
    ParamSgnID{rn}.M_State = getsignalid(tg, [block_path 'Gain_M_State']);
    ParamSgnID{rn}.DSAbrakeCmd = getsignalid(tg, [block_path 'Gain_DSAbrakeCmd']);
    ParamSgnID{rn}.DSAin_MccOn = getsignalid(tg, [block_path 'Gain_DSAin_MccOn']);
    ParamSgnID{rn}.VRDY = getsignalid(tg, [block_path 'Gain_VRDY']);
    ParamSgnID{rn}.DSAAxEtcInfoJ1 = getsignalid(tg, [block_path 'Gain_DSAAxEtcInfoJ1']);
    ParamSgnID{rn}.tDSA_SysInfo2 = getsignalid(tg, [block_path 'Gain_tDSA_SysInfo2']);
    ParamSgnID{rn}.fDSA_SysInfo1 = getsignalid(tg, [block_path 'Gain_fDSA_SysInfo1']);
    ParamSgnID{rn}.SafetyOver = getsignalid(tg, [block_path 'Gain_SafetyOver']);
    
    % Position state
    block_path = ['Scopes/PosState ', num2str(rn), '/'];
    for i = 1:6
        ParamSgnID{rn}.JntPos_deg(i) = getsignalid(tg, [block_path 'Gain_Pos_J', num2str(i), '_deg']);
    end
    
    %% Basic mode related
    block_path = ['Robot No. ', num2str(rn), '/Robot System/ModeSelection/Basic Control Mode/'];
    ParamSgnID{rn}.Brake_Off = getparamid(tg, [block_path 'Brake Off'], 'CurrentSetting');
    ParamSgnID{rn}.Reset_Alarm = getparamid(tg, [block_path 'Reset Alarm'], 'CurrentSetting');
    ParamSgnID{rn}.P_Stop = getparamid(tg, [block_path 'P-STOP'], 'CurrentSetting');
    ParamSgnID{rn}.Hold = getparamid(tg, [block_path 'Hold'], 'CurrentSetting');

    %% Position run & jog related
    block_path = ['Robot No. ', num2str(rn), '/Robot System/ModeSelection/Basic Control Mode/'];
    ParamSgnID{rn}.Tp_Prg_Run = getparamid(tg, [block_path 'TpPrg Run'], 'CurrentSetting');
    ParamSgnID{rn}.TCP_Pos_Run = getparamid(tg, [block_path 'TCP Pos Run'], 'CurrentSetting');
    ParamSgnID{rn}.Tp_Pos_Run = getparamid(tg, [block_path 'TpPos Run'], 'CurrentSetting');
    ParamSgnID{rn}.Jog_Run = getparamid(tg, [block_path 'Jog Run'], 'CurrentSetting');
    ParamSgnID{rn}.JogC_Run = getparamid(tg, [block_path 'JogC Run'], 'CurrentSetting');

    block_path = ['Robot No. ', num2str(rn), '/Robot System/ModeSelection/Pos Run & Jog/'];
    for i = 1:6
        ParamSgnID{rn}.IniJntPos(i) = getparamid(tg, [block_path 'IniJntPos_' num2str(i)], 'Value');
        ParamSgnID{rn}.IniTcpPos(i) = getparamid(tg, [block_path 'IniTcpPos_' num2str(i)], 'Value');
    end
    ParamSgnID{rn}.Jog_OVR = getparamid(tg, [block_path 'jog_ovr'], 'Value');
    ParamSgnID{rn}.Jog_Axis_Num = getparamid(tg, [block_path 'jog_axis_num'], 'Value');
    ParamSgnID{rn}.Jog_Sign = getparamid(tg, [block_path 'jog_sign'], 'Value');
    ParamSgnID{rn}.JogC_OVR = getparamid(tg, [block_path 'jogC_ovr'], 'Value');
    ParamSgnID{rn}.JogC_Axis_Num = getparamid(tg, [block_path 'jogC_axis_num'], 'Value');
    ParamSgnID{rn}.JogC_Sign = getparamid(tg, [block_path 'jogC_sign'], 'Value');
    ParamSgnID{rn}.jogC_vel_cmd = getparamid(tg, [block_path 'jogC_vel_cmd'], 'Value');
    ParamSgnID{rn}.jogC_type_switch = getparamid(tg, [block_path 'jogC_type_switch'], 'Value');
    ParamSgnID{rn}.Record_Pos = getparamid(tg, [block_path 'RecordPos'], 'Value');
    
    %% Force control related
    block_path = ['Robot No. ', num2str(rn), '/Robot System/ModeSelection/Basic Control Mode/'];
    ParamSgnID{rn}.LTT_Run = getparamid(tg, [block_path 'LTT Run'], 'CurrentSetting');
    ParamSgnID{rn}.Tilt_Insert_Run = getparamid(tg, [block_path 'TI Run'], 'CurrentSetting');
    ParamSgnID{rn}.Tilt_Comp_Run = getparamid(tg, [block_path 'TpPrg Run'], 'CurrentSetting');
    ParamSgnID{rn}.Peg_Hole_Insert_Run = getparamid(tg, [block_path 'Force Run'], 'CurrentSetting');
    
    block_path = ['Robot No. ', num2str(rn), '/Robot System/ModeSelection/LTT/'];
    ParamSgnID{rn}.FS_Offset = getparamid(tg, [block_path 'ATI_Offset_Trigger'], 'Value');
    ParamSgnID{rn}.RepForce_Cmd = getparamid(tg, [block_path 'RepFCmd'], 'CurrentSetting');
    ParamSgnID{rn}.LTT_Gain_x = getparamid(tg, [block_path 'LTT_x'], 'Value');
    ParamSgnID{rn}.LTT_Gain_y = getparamid(tg, [block_path 'LTT_y'], 'Value');
    ParamSgnID{rn}.LTT_Gain_z = getparamid(tg, [block_path 'LTT_z'], 'Value');
    ParamSgnID{rn}.LTT_Gain_Rx = getparamid(tg, [block_path 'LTT_Rx'], 'Value');
    ParamSgnID{rn}.LTT_Gain_Ry = getparamid(tg, [block_path 'LTT_Ry'], 'Value');
    ParamSgnID{rn}.LTT_Gain_Rz = getparamid(tg, [block_path 'LTT_Rz'], 'Value');
    ParamSgnID{rn}.RepF_Gain_x = getparamid(tg, [block_path 'Frep_x'], 'Value');
    ParamSgnID{rn}.RepF_Gain_y = getparamid(tg, [block_path 'Frep_y'], 'Value');
    ParamSgnID{rn}.RepF_Gain_z = getparamid(tg, [block_path 'Frep_z'], 'Value');
    
    block_path = ['Robot No. ', num2str(rn), '/Robot System/Measurement/LTT Handle/'];
    ParamSgnID{rn}.LTT_RecordData_On = getparamid(tg, [block_path 'LTT_RecordData_On'], 'Value');
    
    %% Button related
    block_path = ['IO System/Button/PCI-6528 DI Port ', num2str(rn), '/'];
    ParamSgnID{rn}.Green_Button = getsignalid(tg, [block_path, 'p1']); 
    ParamSgnID{rn}.LTT_Button = getsignalid(tg, [block_path, 'p2']); 
    
    %% Gripper related
    block_path = ['Robot No. ', num2str(rn), '/Robot System/Device Control/Gripper Input/'];
    ParamSgnID{rn}.DES_MODE = getparamid(tg, [block_path 'Desired_Mode'], 'Value');
    ParamSgnID{rn}.GPR_MODE = getparamid(tg, [block_path 'GPR_MODE'], 'Value');
    ParamSgnID{rn}.GPR_SETUP = getparamid(tg, [block_path 'GPR_SETUP'], 'CurrentSetting');
    ParamSgnID{rn}.GPR_HOLD = getparamid(tg, [block_path 'GPR_HOLD'], 'CurrentSetting');
    ParamSgnID{rn}.GPR_DRIVE = getparamid(tg, [block_path 'GPR_DRIVE'], 'CurrentSetting');
    ParamSgnID{rn}.GPR_RESET = getparamid(tg, [block_path 'GPR_RESET'], 'CurrentSetting');
    ParamSgnID{rn}.GPR_SVON = getparamid(tg, [block_path 'GPR_SVON'], 'CurrentSetting');
    
    block_path = ['IO System/Gripper System/Gripper ', num2str(rn), '/Gripper Subsystem/'];
    ParamSgnID{rn}.GPR_MODE_OUT = getsignalid(tg, [block_path, 'Bit to Integer Converter']); 
    ParamSgnID{rn}.GPR_BUSY = getsignalid(tg, [block_path, 'Rate Transition17']); 
    ParamSgnID{rn}.GPR_AREA = getsignalid(tg, [block_path, 'Rate Transition16']); 
    ParamSgnID{rn}.GPR_SETON = getsignalid(tg, [block_path, 'Rate Transition15']); 
    ParamSgnID{rn}.GPR_INP = getsignalid(tg, [block_path, 'Rate Transition14']); 
    ParamSgnID{rn}.GPR_SVRE = getsignalid(tg, [block_path, 'Rate Transition13']); 
    ParamSgnID{rn}.GPR_ESTOP = getsignalid(tg, [block_path, 'Rate Transition12']); 
    ParamSgnID{rn}.GPR_ALARM = getsignalid(tg, [block_path, 'Rate Transition24']); 

    %% External motion command related
    block_path = ['Robot No. ', num2str(rn), '/Robot System/ModeSelection/Basic Control Mode/ControllerMode/SetTpPrgToMemory/'];
    ParamSgnID{rn}.const_ExtCmd_vcmd = getparamid(tg, [block_path 'const_ExtCmd_vcmd'], 'Value');
    ParamSgnID{rn}.const_ExtCmd_vcmd_EndCnt= getparamid(tg, [block_path 'const_ExtCmd_vcmd_EndCnt'], 'Value');
    ParamSgnID{rn}.const_ExtCmd_inipos= getparamid(tg, [block_path 'const_ExtCmd_inipos'], 'Value');
    ParamSgnID{rn}.const_ExtCmd_grippercmd= getparamid(tg, [block_path 'const_ExtCmd_grippercmd'], 'Value');
    
end

fprintf('---> Obtained the tunable parameter and signal IDs.\n\n');

end
