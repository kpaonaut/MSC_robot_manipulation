%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       Main File for Teaching with Mouse
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/07/28
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Note: 
% 1. Currently only 1 mouse can be used at a time because Windows cannot 
% distinguish different mouses.
% 2. Here uses while loop to check mouse and keyboard inputs. Use double
% clicking mouse left button (or 'ESC' key) to quit the teaching mode.

function Mouse_Teaching(si, opt)

if nargin < 2,  opt = [];  end
if ~isfield(opt,'robot_idx') || isempty(opt.robot_idx),  opt.robot_idx = si.robot_num;  end   % robot index set for which operations should be conducted (default: last robot)
if ~isfield(opt,'device') || isempty(opt.device),  opt.device = '3DMouse';  end   % device used for teaching (default: 3DMouse; 2DMouse)
if ~isfield(opt,'DataMode') || isempty(opt.DataMode),  opt.DataMode = 0;  end   % 0 - do not save teaching data (default); 1 - save new LTT data but no replay

ParamSgnID = si.ParamSgnID{opt.robot_idx};
tg = SimulinkRealTime.target;
wasStopped = tg_start_stop('start');
wasBrakeOff = brake_on_off(ParamSgnID, 'off');

% Internally use Cartesian space jogging for the mouse teaching motion generation
setparam(tg, ParamSgnID.jogC_type_switch, -1);
setparam(tg, ParamSgnID.JogC_Run, 1);

%% Mouse teaching
% 3D mouse may not work properly. If not, open 3Dconnexion home when MATLAB
% window is active, click Property, reset to initial settings (twice), 
% then close. By this, starting Mouse3D again should get it work.
% For first time use of 3D Mouse, install 3D mouse driver, then set the 
% button function to left - esc, right - shift in the 3Dconnexion home property for MATLAB.
if strcmpi(opt.device, '3DMouse')
    Mouse3D('start');
    % max robot speed in m/s and the rotation matrix to match with tool frame
%     Scale = 0.2 * [0 0 1; -1 0 0; 0 -1 0];    % mouse held at hand
    Scale = 0.2 * [0 1 0; 1 0 0; 0 0 -1];     % mouse mounted on robot 
else    % for other 2D mouse
    PrePntPos = get(0, 'PointerLocation');
    Scale = 1/500;
end
Scale_Gain = 1;
gpr_mode = 'fully open';

finish = false;
while ~finish
    % get the mouse inputs
    if strcmpi(opt.device, '3DMouse')
        out = Mouse3D('get');
        Speed = [Scale * out.pos' / 2500; Scale * out.ang * out.rot' / 2500 * 2] * Scale_Gain;
    else
        CurPntPos = get(0, 'PointerLocation');
        Speed = (CurPntPos - PrePntPos) * Scale * Scale_Gain;
        Speed(abs(Speed)>1) = 0;
        Speed(Speed > 0.1) = 0.1;
        Speed(Speed < -0.1) = -0.1;
        Speed = [0, Speed(1), Speed(2), 0, 0, 0]';
        PrePntPos = CurPntPos;
    end
    
    setparam(tg, ParamSgnID.jogC_vel_cmd, Speed);
    
    % get the keyboard/button inputs
    [key, dclick] = check_kb_input();
    switch key
        case 'tab'    
            if dclick   % if double click, then open/close gripper
                if gripper_mode(gpr_mode) == gripper_mode('fully open')
                    % here set the mouse teaching mode for grabbing cable taping tool
                    %gpr_mode = 'grab cable taping tool firmly';
                    gpr_mode = 'grab cable connector firmly';
                else
                    gpr_mode = 'fully open';
                end
                gripper_action(ParamSgnID, 'DRIVE', gpr_mode);
            end
            % switch to high gain or low gain mode
            % for double clikc case, these will be executed twice.
            if Scale_Gain == 1
                Scale_Gain = 0.3;
            else
                Scale_Gain = 1;
            end
        case 'escape'   
            if dclick   % if double click, then exit the LTT mode
                finish = true;
                check_kb_input('close');
            else        % record the current LTT data
                setparam(tg, ParamSgnID.LTT_RecordData_On, 1);
                pause(0.1);
                setparam(tg, ParamSgnID.LTT_RecordData_On, 0);
            end
    end
end

%% Stop teaching
if strcmpi(opt.device, '3DMouse')
    Mouse3D('stop');
end

setparam(tg, ParamSgnID.JogC_Run, 0);
setparam(tg, ParamSgnID.jogC_type_switch, 1);
brake_on_off(ParamSgnID, 'on', wasBrakeOff);
tg_start_stop('stop', wasStopped);

% save LTT record data without replay
if opt.DataMode
    LTT_Motion_Planning(si, opt);
end
