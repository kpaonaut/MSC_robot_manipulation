%% Wenjie Chen, 2016/06/22, FANUC Corporation
% To unload all loaded Simulink models in order to refresh newest changes
% for model initilization

function unload_all_models()

disp('---> Unloading all loaded Simulink models to refresh newest changes ...');

if bdIsLoaded('LRMate200Experimentor_Trq2DSA'),  close_system('LRMate200Experimentor_Trq2DSA');  end
if bdIsLoaded('LRMate200Experimentor_Servo'),  close_system('LRMate200Experimentor_Servo');  end
if bdIsLoaded('LRMate200Experimentor_RobotState'),  close_system('LRMate200Experimentor_RobotState');  end
if bdIsLoaded('LRMate200Experimentor_Robot_Copy'),  close_system('LRMate200Experimentor_Robot_Copy');  end
if bdIsLoaded('LRMate200Experimentor_Robot'),  close_system('LRMate200Experimentor_Robot');  end
if bdIsLoaded('LRMate200Experimentor_ModeController'),  close_system('LRMate200Experimentor_ModeController');  end
if bdIsLoaded('LRMate200Experimentor_Dual'),  close_system('LRMate200Experimentor_Dual');  end
if bdIsLoaded('LRMate200Experimentor_Single'),  close_system('LRMate200Experimentor_Single');  end

fprintf('---> All loaded Simulink models are unloaded.\n\n');
