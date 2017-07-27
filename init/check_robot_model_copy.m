%% Wenjie Chen, 2016/06/22, FANUC Corporation

% To make sure two robot models are exactly the same.

% LRMate200Experimentor_Robot.slx and LRMate200Experimentor_Robot_Copy.slx 
% should be exactly the same. These two do not share the same file because 
% somehow SLRT panel couldn't control each robot individually with the same
% robot model file.

% Output: status - 
%               0: models were not the same, and failed to be updated
%               1: models were not the same, and are now updated successfully
%               2: default: models were already the same

function status = check_robot_model_copy(opt)
status = 2;     % default: models are the same

if nargin < 1,  opt = [];  end
if ~isfield(opt,'is_dual_arm') || isempty(opt.is_dual_arm),  opt.is_dual_arm = true;  end
if ~isfield(opt,'relPath2Experimentor') || isempty(opt.relPath2Experimentor),  opt.relPath2Experimentor = 'experimentor\';  end

if opt.is_dual_arm
    file_name_main = 'LRMate200Experimentor_Dual.slx';
else
    file_name_main = 'LRMate200Experimentor_Single.slx';
end

file_name_1 = 'LRMate200Experimentor_Robot.slx';
file_name_2 = 'LRMate200Experimentor_Robot_Copy.slx';

% visdiff(file_name_1,file_name_2);
% f1 = dir([opt.relPath2Experimentor file_name_1]);
% f2 = dir([opt.relPath2Experimentor file_name_2]);
% isequal(f1, f2)

fileID = fopen(file_name_1);
A = fread(fileID);
fclose(fileID);

fileID = fopen(file_name_2);
B = fread(fileID);
fclose(fileID);

if ~isequal(A, B)
    disp('---> Robot Simulink models are not the same! Now updating the copy model to match the robot model ...');
    
    if bdIsLoaded(file_name_2(1:end-4)),  close_system(file_name_2);  end
    if bdIsLoaded(file_name_main(1:end-4)),  close_system(file_name_main);  end
    
    [status, message] = copyfile([opt.relPath2Experimentor file_name_1], [opt.relPath2Experimentor file_name_2], 'f');     % status is 1 for success and 0 for failure.
    if ~status,  error(message);  end
    
    fprintf('---> Robot Simulink models are updated to be the same.\n\n');
end

end
